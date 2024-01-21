#include <nt36xxx\ntfwupdate.h>
#include <ntfwupdate.tmh>

//Firmware update functions copied from linux driver

PUCHAR fwbuf = NULL;

struct nvt_ts_bin_map {
    char name[12];
    unsigned int BIN_addr;
    unsigned int SRAM_addr;
    unsigned int size;
    unsigned int crc;
};

static struct nvt_ts_bin_map* bin_map;

//NT36523 memory map
unsigned int EVENT_BUF_ADDR = 0x2FE00;
/* Phase 2 Host Download */
unsigned int BOOT_RDY_ADDR = 0x3F10D;
unsigned int TX_AUTO_COPY_EN = 0x3F7E8;
/* BLD CRC */
unsigned int ILM_LENGTH_ADDR = 0x3F118;
unsigned int DLM_LENGTH_ADDR = 0x3F130;
unsigned int ILM_DES_ADDR = 0x3F128;
unsigned int DLM_DES_ADDR = 0x3F12C;
unsigned int G_ILM_CHECKSUM_ADDR = 0x3F100;
unsigned int G_DLM_CHECKSUM_ADDR = 0x3F104;
unsigned int BLD_CRC_EN_ADDR = 0x3F30E;

int nt36xxx_set_page(IN SPB_CONTEXT* SpbContext, IN unsigned int pageaddr)
{
    unsigned char addr = 0xFF; //set index/page/addr command
    unsigned char buf[4] = { 0 };

    buf[0] = (pageaddr >> 15) & 0xFF;
    buf[1] = (pageaddr >> 7) & 0xFF;

    return SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(addr), buf, 2);
}

int nt36xxx_write_addr(IN SPB_CONTEXT* SpbContext, unsigned int addr, unsigned char data)
{
    int ret = 0;

    unsigned char buf[1];

    buf[0] = data;

    //---set xdata index---
    ret = nt36xxx_set_page(SpbContext, addr);
    if (ret) {
        return ret;
    }

    //---write data to index---
    ret = SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(addr & 0x7F), buf, 1);
    if (ret) {
        return ret;
    }

    return ret;
}

int nt36xxx_eng_reset(IN SPB_CONTEXT* SpbContext)
{
    LARGE_INTEGER Interval;
    int ret;

    ret = nt36xxx_write_addr(SpbContext, ENG_RST_ADDR, 0x5A);
    if (ret)
        return ret;

    Interval.QuadPart = RELATIVE(MILLISECONDS(10));
    KeDelayExecutionThread(KernelMode, TRUE, &Interval);

    return ret;
}

int nt36xxx_bootloader_reset(IN SPB_CONTEXT* SpbContext)
{
    LARGE_INTEGER Interval;
    int ret;

    ret = nt36xxx_write_addr(SpbContext, SWRST_N8_ADDR, NT36XXX_CMD_BOOTLOADER_RESET);
    if (ret)
        return ret;

    /* MCU has to reboot from bootloader: this is the typical boot time */
    Interval.QuadPart = RELATIVE(MILLISECONDS(5));
    KeDelayExecutionThread(KernelMode, TRUE, &Interval);

    ret = nt36xxx_write_addr(SpbContext, SPI_RD_FAST_ADDR, 0x00);
    if (ret)
        return ret;

    return ret;
}

static unsigned int byte_to_word(const unsigned char* data)
{
    return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

static void nvt_set_bld_crc_bank(unsigned int DES_ADDR, unsigned int SRAM_ADDR,
    unsigned int LENGTH_ADDR, unsigned int size,
    unsigned int G_CHECKSUM_ADDR, unsigned int crc, SPB_CONTEXT* SpbContext)
{
    /* write destination address */
    nt36xxx_set_page(SpbContext, DES_ADDR);
    fwbuf[0] = (SRAM_ADDR) & 0xFF;
    fwbuf[1] = (SRAM_ADDR >> 8) & 0xFF;
    fwbuf[2] = (SRAM_ADDR >> 16) & 0xFF;
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(DES_ADDR & 0x7F), fwbuf, 3);

    /* write length */
    //nvt_set_page(LENGTH_ADDR);
    fwbuf[0] = (size) & 0xFF;
    fwbuf[1] = (size >> 8) & 0xFF;
    fwbuf[2] = (size >> 16) & 0x01;
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(LENGTH_ADDR & 0x7F), fwbuf, 3);

    /* write golden dlm checksum */
    //nvt_set_page(G_CHECKSUM_ADDR);
    fwbuf[0] = (crc) & 0xFF;
    fwbuf[1] = (crc >> 8) & 0xFF;
    fwbuf[2] = (crc >> 16) & 0xFF;
    fwbuf[3] = (crc >> 24) & 0xFF;
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(G_CHECKSUM_ADDR & 0x7F), fwbuf, 4);

    return;
}

void nvt_bld_crc_enable(SPB_CONTEXT* SpbContext)
{
    unsigned char buf[20] = { 0 };

    //---set xdata index to BLD_CRC_EN_ADDR---
    nt36xxx_set_page(SpbContext, BLD_CRC_EN_ADDR);

    //---read data from index---
    buf[0] = 0xFF;
    SpbReadDataSynchronously(SpbContext, SPI_READ_MASK(BLD_CRC_EN_ADDR & 0x7F), buf, 20, FALSE);

    //---write data to index---
    buf[0] = buf[16] | (0x01 << 7);
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(BLD_CRC_EN_ADDR & 0x7F), buf, 1);
}

void nvt_fw_crc_enable(SPB_CONTEXT* SpbContext)
{
    unsigned char buf[4] = { 0 };

    //---set xdata index to EVENT BUF ADDR---
    nt36xxx_set_page(SpbContext, EVENT_BUF_ADDR);

    //---clear fw reset status---
    buf[0] = 0x00;
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(NT36XXX_EVT_RESET_COMPLETE & 0x7F), buf, 1);

    //---enable fw crc---
    buf[0] = 0xAE;	//enable fw crc command
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(NT36XXX_EVT_HOST_CMD & 0x7F), buf, 1);
}

void nvt_boot_ready(SPB_CONTEXT* SpbContext)
{
    LARGE_INTEGER delay = { 0 };
    
    //---write BOOT_RDY status cmds---
    nt36xxx_write_addr(SpbContext, BOOT_RDY_ADDR, 1);
    
    delay.QuadPart = RELATIVE(MILLISECONDS(5));
	KeDelayExecutionThread(KernelMode, TRUE, &delay);
}

void disable_pen_input_device(SPB_CONTEXT* SpbContext, BOOLEAN disable)
{
    unsigned char buf[4] = { 0 };
    
    LARGE_INTEGER delay = { 0 };
    
    delay.QuadPart = RELATIVE(MILLISECONDS(35));
	KeDelayExecutionThread(KernelMode, TRUE, &delay);

    //---set xdata index to EVENT BUF ADDR---
    nt36xxx_set_page(SpbContext, EVENT_BUF_ADDR | NT36XXX_EVT_HOST_CMD);

    //---clear fw reset status---
    buf[0] = 0x7B;
    buf[1] = disable;
    SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(NT36XXX_EVT_HOST_CMD & 0x7F), buf, 2);
}

NTSTATUS
NVTLoadFirmwareFile(WDFDEVICE FxDevice, SPB_CONTEXT* SpbContext) {
    HANDLE handle;
    NTSTATUS ntstatus;
    IO_STATUS_BLOCK ioStatusBlock;
    LARGE_INTEGER byteOffset;

    UNICODE_STRING     uniName;
    UNICODE_STRING     ntFWPath;
    OBJECT_ATTRIBUTES  objAttr;

    WDFKEY          hKey = NULL;

    UNICODE_STRING  NVTFWFilePathKey;
    WDFSTRING NVTFWFilePath;

    GUID efiGUID = { 0x9042a9de, 0x23dc, 0x4a38, { 0x96, 0xfb, 0x7a, 0xde, 0xd0, 0x80, 0x51, 0x6a } };
    UNICODE_STRING efiVar;
    unsigned int outVar[32];
    ULONG outVarLen;

    RtlInitUnicodeString(&efiVar, L"UEFIDisplayInfo");

    outVarLen = sizeof(outVar);

    ntstatus = ExGetFirmwareEnvironmentVariable(&efiVar, &efiGUID, &outVar, &outVarLen, NULL);
    if (NT_SUCCESS(ntstatus)) {
        if(outVar[10] == 0x25) {
            RtlInitUnicodeString(&NVTFWFilePathKey, L"NVTFWImageHuaxingPath");
        }
        else {
            RtlInitUnicodeString(&NVTFWFilePathKey, L"NVTFWImageTianmaPath");
        }
    }
    else
        RtlInitUnicodeString(&NVTFWFilePathKey, L"NVTFWImageHuaxingPath");

    PCHAR buffer = (PCHAR)ExAllocatePoolWithTag(
        NonPagedPool,
        FWBUFFER_SIZE,
        TOUCH_POOL_TAG
    );

    //FILE_STANDARD_INFORMATION fileInfo = { 0 };

    WdfStringCreate(
        NULL,
        WDF_NO_OBJECT_ATTRIBUTES,
        &NVTFWFilePath);

    ntstatus = WdfDeviceOpenRegistryKey(FxDevice,
        PLUGPLAY_REGKEY_DRIVER,
        GENERIC_READ,
        WDF_NO_OBJECT_ATTRIBUTES,
        &hKey);

    if (NT_SUCCESS(ntstatus)) {

        ntstatus = WdfRegistryQueryString(hKey, &NVTFWFilePathKey, NVTFWFilePath);

        WdfRegistryClose(hKey);
    }

    WdfStringGetUnicodeString(NVTFWFilePath, &uniName);

    ntFWPath.Length = 0;
    ntFWPath.MaximumLength = 256 * sizeof(WCHAR);
    ntFWPath.Buffer = ExAllocatePoolWithTag(NonPagedPool, ntFWPath.MaximumLength, TOUCH_POOL_TAG);

    RtlAppendUnicodeToString(&ntFWPath, L"\\DosDevices\\");
    RtlAppendUnicodeStringToString(&ntFWPath, &uniName);

    InitializeObjectAttributes(&objAttr, &ntFWPath,
        OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE,
        NULL, NULL);


    // Do not try to perform any file operations at higher IRQL levels.
    // Instead, you may use a work item or a system worker thread to perform file operations.

    if (KeGetCurrentIrql() != PASSIVE_LEVEL)
        return STATUS_INVALID_DEVICE_STATE;

    ntstatus = ZwCreateFile(&handle,
        GENERIC_READ,
        &objAttr, &ioStatusBlock, NULL,
        FILE_ATTRIBUTE_NORMAL,
        0,
        FILE_OPEN,
        FILE_SYNCHRONOUS_IO_NONALERT,
        NULL, 0);

    /*ntstatus = ZwQueryInformationFile(
        handle,
        &ioStatusBlock,
        &fileInfo,
        sizeof(fileInfo),
        FileStandardInformation
    );*/

    if (NT_SUCCESS(ntstatus)) {
        Trace(
            TRACE_LEVEL_INFORMATION,
            TRACE_INTERRUPT,
            "Found: novatek_nt36523_fw.bin");

        byteOffset.QuadPart = 0;
        ntstatus = ZwReadFile(handle, NULL, NULL, NULL, &ioStatusBlock,
            buffer, FWBUFFER_SIZE, &byteOffset, NULL);
        ZwClose(handle);
    }

    //nvt_bin_header_parser
    static unsigned int partition = 0;
    static unsigned char ilm_dlm_num = 2;
    static unsigned char cascade_2nd_header_info = 0;

    unsigned int list = 0;
    unsigned int pos = 0x00;
    unsigned int end = 0x00;
    unsigned char info_sec_num = 0;
    unsigned char ovly_sec_num = 0;
    unsigned char ovly_info = 0;
    unsigned char find_bin_header = 0;

    end = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
    cascade_2nd_header_info = (buffer[0x20] & 0x02) >> 1;
    Trace(
        TRACE_LEVEL_INFORMATION,
        TRACE_INTERRUPT,
        "cascade_2nd_header_info = %d", cascade_2nd_header_info);

    if (cascade_2nd_header_info) {
        pos = 0x30;	// info section start at 0x30 offset
        while (pos < (end / 2)) {
            info_sec_num++;
            pos += 0x10;	/* each header info is 16 bytes */
        }

        info_sec_num = info_sec_num + 1; //next header section
    }
    else {
        pos = 0x30;	// info section start at 0x30 offset
        while (pos < end) {
            info_sec_num++;
            pos += 0x10;	/* each header info is 16 bytes */
        }
    }

    /*
     * Find the DLM OVLY section
     * [0:3] Overlay Section Number
     * [4]   Overlay Info
     */
    ovly_info = (buffer[0x28] & 0x10) >> 4;
    ovly_sec_num = (ovly_info) ? (buffer[0x28] & 0x0F) : 0;

    /*
     * calculate all partition number
     * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
     */
    partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
    Trace(
        TRACE_LEVEL_INFORMATION,
        TRACE_INTERRUPT,
        "ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
        ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

    /* allocated memory for header info */
    bin_map = (struct nvt_ts_bin_map*)(PUCHAR)ExAllocatePoolWithTag(
        NonPagedPool,
        (partition + 1) * sizeof(struct nvt_ts_bin_map),
        TOUCH_POOL_TAG
    );
    if (bin_map == NULL) {
        Trace(
            TRACE_LEVEL_INFORMATION,
            TRACE_INTERRUPT,
            "kzalloc for bin_map failed!\n");
    }

    for (list = 0; list < partition; list++) {
        /*
         * [1] parsing ILM & DLM header info
         * BIN_addr : SRAM_addr : size (12-bytes)
         * crc located at 0x18 & 0x1C
         */
        if (list < ilm_dlm_num) {
            bin_map[list].BIN_addr = byte_to_word((unsigned char*)&buffer[0 + list * 12]);
            bin_map[list].SRAM_addr = byte_to_word((unsigned char*)&buffer[4 + list * 12]);
            bin_map[list].size = byte_to_word((unsigned char*)&buffer[8 + list * 12]);
            bin_map[list].crc = byte_to_word((unsigned char*)&buffer[0x18 + list * 4]);

            if (list == 0)
                sprintf(bin_map[list].name, "ILM");
            else if (list == 1)
                sprintf(bin_map[list].name, "DLM");
        }

        /*
         * [2] parsing others header info
         * SRAM_addr : size : BIN_addr : crc (16-bytes)
         */
        if ((list >= ilm_dlm_num) && ((unsigned char)list < (ilm_dlm_num + info_sec_num))) {
            if (find_bin_header == 0) {
                /* others partition located at 0x30 offset */
                pos = 0x30 + (0x10 * (list - ilm_dlm_num));
            }
            else if (find_bin_header && cascade_2nd_header_info) {
                /* cascade 2nd header info */
                pos = end - 0x10;
            }

            bin_map[list].SRAM_addr = byte_to_word((unsigned char*)&buffer[pos]);
            bin_map[list].size = byte_to_word((unsigned char*)&buffer[pos + 4]);
            bin_map[list].BIN_addr = byte_to_word((unsigned char*)&buffer[pos + 8]);
            bin_map[list].crc = byte_to_word((unsigned char*)&buffer[pos + 12]);

            /* detect header end to protect parser function */
            if ((bin_map[list].BIN_addr < end) && (bin_map[list].size != 0)) {
                sprintf(bin_map[list].name, "Header");
                find_bin_header = 1;
            }
            else {
                sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
            }
        }

        /*
         * [3] parsing overlay section header info
         * SRAM_addr : size : BIN_addr : crc (16-bytes)
         */
        if ((unsigned char)list >= (ilm_dlm_num + info_sec_num)) {
            /* overlay info located at DLM (list = 1) start addr */
            pos = bin_map[1].BIN_addr + (0x10 * (list - ilm_dlm_num - info_sec_num));

            bin_map[list].SRAM_addr = byte_to_word((unsigned char*)&buffer[pos]);
            bin_map[list].size = byte_to_word((unsigned char*)&buffer[pos + 4]);
            bin_map[list].BIN_addr = byte_to_word((unsigned char*)&buffer[pos + 8]);
            bin_map[list].crc = byte_to_word((unsigned char*)&buffer[pos + 12]);

            sprintf(bin_map[list].name, "Overlay-%d", (list - ilm_dlm_num - info_sec_num));
        }

        /* BIN size error detect */
        if ((bin_map[list].BIN_addr + bin_map[list].size) > FWBUFFER_SIZE) {
            Trace(
                TRACE_LEVEL_INFORMATION,
                TRACE_INTERRUPT, "access range (0x%08X to 0x%08X) is larger than bin size!\n",
                bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
        }

        //		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X), CRC (0x%08X)\n",
        //				list, bin_map[list].name,
        //				bin_map[list].SRAM_addr, bin_map[list].size,  bin_map[list].BIN_addr, bin_map[list].crc);
    }
    //nvt_bin_header_parser end

    //nvt_download_init
    fwbuf = (PUCHAR)ExAllocatePoolWithTag(
        NonPagedPool,
        NVT_TRANSFER_LEN + 2,
        TOUCH_POOL_TAG
    );

    //nvt_download_firmware_hw_crc
    nt36xxx_bootloader_reset(SpbContext);

    /* [0] ILM */
    /* write register bank */
    nvt_set_bld_crc_bank(ILM_DES_ADDR, bin_map[0].SRAM_addr,
        ILM_LENGTH_ADDR, bin_map[0].size,
        G_ILM_CHECKSUM_ADDR, bin_map[0].crc, SpbContext);

    /* [1] DLM */
    /* write register bank */
    nvt_set_bld_crc_bank(DLM_DES_ADDR, bin_map[1].SRAM_addr,
        DLM_LENGTH_ADDR, bin_map[1].size,
        G_DLM_CHECKSUM_ADDR, bin_map[1].crc, SpbContext);

    nt36xxx_write_addr(SpbContext, TX_AUTO_COPY_EN, 0x69);

    //nvt_write_firmware
    list = 0;//unsigned int list = 0;
    char* name;
    unsigned int BIN_addr, SRAM_addr, size;
    int ret = 0;

    memset(fwbuf, 0, (NVT_TRANSFER_LEN + 1));

    for (list = 0; list < partition; list++) {
        /* initialize variable */
        SRAM_addr = bin_map[list].SRAM_addr;
        size = bin_map[list].size;
        BIN_addr = bin_map[list].BIN_addr;
        name = bin_map[list].name;

        //		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X)\n",
        //				list, name, SRAM_addr, size, BIN_addr);

                /* Check data size */
        if ((BIN_addr + size) > FWBUFFER_SIZE) {
            Trace(
                TRACE_LEVEL_INFORMATION,
                TRACE_INTERRUPT, "access range (0x%08X to 0x%08X) is larger than bin size!\n",
                BIN_addr, BIN_addr + size);
        }

        /* ignore reserved partition (Reserved Partition size is zero) */
        if (!size)
            continue;
        else
            size = size + 1;

        /* write data to SRAM */
        //nvt_write_sram
        ret = 0;
        int i = 0;
        unsigned short len = 0;
        int count = 0;

        if (size % NVT_TRANSFER_LEN)
            count = (size / NVT_TRANSFER_LEN) + 1;
        else
            count = (size / NVT_TRANSFER_LEN);

        for (i = 0; i < count; i++) {
            len = (unsigned short)((size < NVT_TRANSFER_LEN) ? size : NVT_TRANSFER_LEN);

            //---set xdata index to start address of SRAM---
            ret = nt36xxx_set_page(SpbContext, SRAM_addr);
            if (ret) {
                Trace(
                    TRACE_LEVEL_INFORMATION,
                    TRACE_INTERRUPT, "set page failed, ret = %d\n", ret);
                return ret;
            }

            //---write data into SRAM---
            memcpy(fwbuf, &buffer[BIN_addr], len);	//payload
            ret = SpbWriteDataSynchronously(SpbContext, SPI_WRITE_MASK(SRAM_addr & 0x7F), fwbuf, len);
            if (ret) {
                Trace(
                    TRACE_LEVEL_INFORMATION,
                    TRACE_INTERRUPT, "write to sram failed, ret = %d\n", ret);
                return ret;
            }

            SRAM_addr += NVT_TRANSFER_LEN;
            BIN_addr += NVT_TRANSFER_LEN;
            size -= NVT_TRANSFER_LEN;
        }
    }

    nvt_bld_crc_enable(SpbContext);

    nvt_fw_crc_enable(SpbContext);

    nvt_boot_ready(SpbContext);
    
    disable_pen_input_device(SpbContext, FALSE);
    
    ExFreePoolWithTag((PVOID)buffer, TOUCH_POOL_TAG);
    ExFreePoolWithTag((PVOID)bin_map, TOUCH_POOL_TAG);
    ExFreePoolWithTag((PVOID)fwbuf, TOUCH_POOL_TAG);

    return ntstatus;
}
