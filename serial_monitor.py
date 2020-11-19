"""
Simple display program for fast streaming serial diata

"""

import serial
import re
import curses


def regex2nums(regex, data):
    nums = regex.search(data)
    return [float(nums.group(1+n)) for n in range(regex.groups)]


ser = serial.Serial('/dev/ttyUSB0', 115200)


gyro_regex = re.compile("G: ([0-9\-\.]+), ([0-9\-\.]+), ([0-9\-\.]+)")
accel_regex = re.compile("A: ([0-9\-\.]+), ([0-9\-\.]+), ([0-9\-\.]+)")
mag_regex = re.compile("M: ([0-9\-\.]+), ([0-9\-\.]+), ([0-9\-\.]+)")
eulers_regex = re.compile("P, R, Y: ([0-9\-\.]+), ([0-9\-\.]+), ([0-9\-\.]+)")

accel_kf_regex = re.compile("KFA: ([0-9\-\.]+), ([0-9\-\.]+), ([0-9\-\.]+)")
gyro_kf_regex = re.compile("KFG: ([0-9\-\.]+), ([0-9\-\.]+), ([0-9\-\.]+)")

press_abs = re.compile("PA: ([0-9\-\.]+), ([0-9\-\.]+)")
press_diff = re.compile("PD: ([0-9\-\.]+), ([0-9\-\.]+)")


screen = curses.initscr()
screen.nodelay(True)

run_bool = True
raw_mode = True

while run_bool:
    data = ser.read(1024).decode()
    if raw_mode:
        print(data)
    else:

        gyros = regex2nums(gyro_regex, data)
        accels = regex2nums(accel_regex, data)
        kfgyros = regex2nums(gyro_kf_regex, data)
        kfaccels = regex2nums(accel_kf_regex, data)
        eulers = regex2nums(eulers_regex, data)
        mags = regex2nums(mag_regex, data)

        pa = regex2nums(press_abs, data)
        pd = regex2nums(press_diff, data)

        screen.addstr(0, 0, f"Raw gyros:  {gyros[0]:9.3f}{gyros[1]:9.3f}{gyros[2]:9.3f}")
        screen.addstr(1, 0, f"KF gyros:   {kfgyros[0]:9.3f}{kfgyros[1]:9.3f}{kfgyros[2]:9.3f}")
        screen.addstr(3, 0, f"Raw accels: {accels[0]:9.3f}{accels[1]:9.3f}{accels[2]:9.3f}")
        screen.addstr(4, 0, f"KF accels:  {kfaccels[0]:9.3f}{kfaccels[1]:9.3f}{kfaccels[2]:9.3f}")
        screen.addstr(6, 0, f"Mag:        {mags[0]:9.3f}{mags[1]:9.3f}{mags[2]:9.3f}")

        screen.addstr(8, 0, f"Press Abs:  {pa[0]:9.3f}{pa[1]:9.3f}")
        screen.addstr(9, 0, f"Press D:    {pd[0]:9.3f}{pd[1]:9.3f}")
        screen.addstr(11, 0, f"R, P, H:    {eulers[1]:9.3f}{eulers[0]:9.3f}{eulers[2]:9.3f}")

        screen.refresh()
        c = screen.getch()
        if c != -1:
            curses.endwin()
            run_bool = False


