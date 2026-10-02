"""把本工程自带库里的 WiFi 等待超时从 1000ms 改成 2ms（保持 GBK 编码）。

原因：wifi_spi_send_buffer / wifi_spi_read_buffer 内部调用 wifi_spi_wait_idle(OTHER_TIME_OUT)，
      wait_idle 是"死等 INT 引脚变高"的忙循环。原值 1000 意味着模块一忙就阻塞整整 1 秒。
"""
P = r"D:/jisuyueye9car/A_zjc_project/4.motor_wifi_verify_cyt4bb7/libraries/zf_device/zf_device_wifi_spi.c"

OLD = '#define OTHER_TIME_OUT              1000        // 单位毫秒'
NEW = ('#define OTHER_TIME_OUT              2           // 单位毫秒\n'
       '// [本项目改动] 原值 1000：模块一忙就阻塞整整 1 秒，会把主循环拖垮。\n'
       '//   改成 2 之后最坏只阻塞 2ms；上层 motor_link 已用 INT 引脚做"忙就丢帧"的保护。\n'
       '//   还原方法：把这一行的 2 改回 1000，并删掉上面三行注释。')

txt = open(P, 'rb').read().decode('gbk', errors='strict')
if OLD not in txt:
    if 'OTHER_TIME_OUT              2 ' in txt:
        print('已经改过了，跳过')
    else:
        raise SystemExit('没找到目标宏，也没有改动痕迹 —— 先人工看一眼文件头')
else:
    txt = txt.replace(OLD, NEW, 1)
    open(P, 'wb').write(txt.encode('gbk'))
    print('OTHER_TIME_OUT 已改为 2')
