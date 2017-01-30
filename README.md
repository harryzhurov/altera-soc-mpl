# altera-soc-mpl

## Инструкция

1. Создать проект в Quartus.
1. Запустить Qsys.
1. Добавить компонент hps. Перейти в режим редактирования этого компонента.
1. Настроить клоки. Добавить UART0 для поддержки печати в консоль, добавить QSPI (пины, на которые мапить, посмотреть по схеме кита).
1. Отредактировать параметры SDRAM, начать можно с шаблона JEDEC.
1. Сохранить. Сгенерировать HDL.
1. В top-level исходном файле проекта добавить порты для всех компонентов hps - памяти, UART, QSPI, имена портов можно взять в сгенерированных файлах.
1. Запустить Analysis & Syntesis.
1. Открыть в Quartus Tcl console, запустить ` hps_sdram_p0_pin_assignments.tcl`, это производит настройку пинов компонента.
1. Запустить полную сборку. Сборка на этапе ассемблера создать директорию `hps_isw_handoff`, которая содержит информацию о настроках внутренностей hps компонента и код для теста SDRAM. Зачем для генерации пререквезитов для процессорной части производить синтез проекта FPGA - индусская загадка.
1. Запустить bs-editor ($EDS_SOC/embedded/host_tools/altera/preloadergen), в нём:
    1. File->New BSP
    1. Указать путь до внутреней директории в `hps_isw_handoff`.
    1. Установить требуемые параметры - например, загрузку из QSPI.
    1. Нажать кнопку `Generate`.
    1. Это создаст директорию `software/spl_bsp` и её содержимое.
1. Скопировать в корень проекта заготовку MPL: из файла $EDS_SOC/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL.tar.gz
1. Отредактировать внутри этой заготовки файлы для сборки: config.mk и Makefile: указать источник загрузки, тип дивайса, путь к тулчейну и к директориям со сгенерированными файлами - тем, которые сгенерировал Quartus и bsp-editor.
1. Запустить сборку: make.
1. В DS-5 создать bare-metal проект и указать .axf файл (на самом деле это .elf)


## Проблемы 

1. Указать путь к тулчейну `SOCEDS_DEST_ROOT` недостаточно, путь к `mkpimage`в `Makefile` определён без указания этого макроса, поэтому нужно это место откорректировать.
1. На SoCkit Arrow стоит QSPI Micron `N25Q00AA`, чтобы прошивка MPL собралась корректно, нужно подключить использование это микросхемы, по умолчанию подключена `N25Q512A`. Вариант: указать компилятору `-DALT_QSPI_SUPPORT_ALL`, тогда подключается поддержка всех микросхем (и появляется пачка предупреждений о переопределении макросов - инусокод).
1. Запустить тест SDRAM (калибровка/тренировка) не удалось. При попытке собрать, указав в файле `sequencer_defines.h` значения макроса `RUNTIME_CAL_REPORT 0` и в командной строке `-DHPS_HW_SERIAL_SUPPORT` (чтобы выводило отчёт), линкер выдаёт ошибки по поводу ненайденных функций

```
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-sbrkr.o): In function `_sbrk_r':
sbrkr.c:(.text+0x18): undefined reference to `_sbrk'
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-writer.o): In function `_write_r':
writer.c:(.text+0x24): undefined reference to `_write'
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-closer.o): In function `_close_r':
closer.c:(.text+0x18): undefined reference to `_close'
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-fstatr.o): In function `_fstat_r':
fstatr.c:(.text+0x20): undefined reference to `_fstat'
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-isattyr.o): In function `_isatty_r':
isattyr.c:(.text+0x18): undefined reference to `_isatty'
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-lseekr.o): In function `_lseek_r':
lseekr.c:(.text+0x24): undefined reference to `_lseek'
/opt/cad/altera/16.1/embedded/host_tools/mentor/gnu/arm/baremetal/bin/../lib/gcc/arm-altera-eabi/5.2.0/../../../../arm-altera-eabi/lib/libc.a(lib_a-readr.o): In function `_read_r':
readr.c:(.text+0x24): undefined reference to `_read'
```

и самое главное:

```
arm-altera-eabi/bin/ld: mpl_C5_q.axf section `.text.rw_mgr_mem_calibrate_read_test_patterns_all_ranks' will not fit in region `prog_mem'
arm-altera-eabi/bin/ld: region `prog_mem' overflowed by 21564 bytes
```

т.е. код не влезает в размер памяти. По видимому, весь этот код расчитан на запуск в составе UBOOT, т.е. из SDRAM.
