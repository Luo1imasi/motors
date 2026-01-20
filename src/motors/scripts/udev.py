#!/usr/bin/env python3

import subprocess
import os
import time
import shutil
import glob
import sys
import re

def clear_screen():
    """清空终端屏幕"""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_header(title="ROS2 设备 UDEV 规则管理器"):
    """打印程序标题"""
    clear_screen()
    print("=" * 60)
    print(f"              {title}")
    print("=" * 60)
    if "CAN" not in title:
        print("此工具可帮助您创建和管理 TTY 设备的 udev 规则")
    else:
        print("此工具可帮助您创建和管理 CAN 设备的 udev 规则")
    print("-" * 60)

# ==============================================================================
# TTY 模式 (原始功能)
# ==============================================================================

def get_tty_device_descriptor(device_path):
    """获取 TTY 设备的内核描述符 (KERNELS==)"""
    try:
        result = subprocess.run(
            ["udevadm", "info", "--attribute-walk", "--name", device_path],
            capture_output=True, text=True, check=True
        )
        kernels_lines = [line for line in result.stdout.split('\n') if 'KERNELS==' in line]
        if len(kernels_lines) > 1:
            return kernels_lines[1].split('"')[1]  # 获取第二个 KERNELS== 的值
        elif len(kernels_lines) == 1:
             return kernels_lines[0].split('"')[1] # 备用
        else:
            print(f"未找到足够的 KERNELS== 行: {kernels_lines}")
    except subprocess.CalledProcessError as e:
        print(f"获取设备描述符时出错: {e}")
    return None

def create_tty_udev_rule(device_name, descriptor, output_dir):
    """创建 TTY (symlink) udev规则文件"""
    rule = f'KERNELS=="{descriptor}", MODE:="0666", SYMLINK+="{device_name}"'
    rule_file = os.path.join(output_dir, f'{device_name}.rules')
    try:
        with open(rule_file, 'w') as f:
            f.write(rule + '\n')
        print(f"已创建 {device_name} 的udev规则: {rule_file}")
        return True
    except IOError as e:
        print(f"写入 {device_name} 的udev规则时出错: {e}")
    except Exception as e:
        print(f"写入 {device_name} 的udev规则时发生意外错误: {e}")
    return False

def detect_tty_device(device_name, device_symlink):
    """交互式检测 TTY 设备"""
    print(f"\n正在设置 {device_name} (TTY) 的udev规则...")
    print(f"该设备将使用符号链接: /dev/{device_symlink}")
    
    print("\n请选择获取设备标识符的方式:")
    print("1. 自动检测 (插拔设备自动识别)")
    print("2. 手动输入 (直接输入已知的设备路径)")
    print("0. 返回上级菜单")
    
    while True:
        choice = input("\n请选择 [0-2]: ").strip()
        
        if choice == "0":
            return None, None
        elif choice == "1":
            return detect_tty_by_plugin(device_name, device_symlink)
        elif choice == "2":
            return detect_tty_by_manual_input(device_name, device_symlink)
        else:
            print("无效选项，请重新选择!")

def detect_tty_by_manual_input(device_name, device_symlink):
    """手动输入 TTY 设备路径获取标识符"""
    print(f"\n手动输入模式 - {device_name}")
    print("提示: 可以使用以下命令查看当前可用的串口设备:")
    print("  ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
    print("  dmesg | grep tty")
    print()
    
    while True:
        device_path = input("请输入设备路径 (例如: /dev/ttyUSB0) (输入0返回): ").strip()
        
        if not device_path:
            print("设备路径不能为空!")
            continue
        
        if device_path == "0":
            return None, None
        
        if not os.path.exists(device_path):
            print(f"设备路径 {device_path} 不存在!")
            retry = input("是否重新输入? (y/n): ")
            if retry.lower() != 'y':
                return None, None
            continue
        
        if not (device_path.startswith('/dev/ttyUSB') or device_path.startswith('/dev/ttyACM')):
            print("警告: 输入的设备路径可能不是 TTY 设备")
            confirm = input("是否继续? (y/n): ")
            if confirm.lower() != 'y':
                continue
        
        print(f"\n正在获取设备 {device_path} 的标识符...")
        descriptor = get_tty_device_descriptor(device_path)
        
        if descriptor:
            print(f"成功获取设备标识符: {descriptor}")
            confirm = input(f"确认使用此设备 {device_path} 作为 {device_name}? (y/n): ")
            if confirm.lower() == 'y':
                return device_symlink, descriptor
            else:
                retry = input("是否重新输入设备路径? (y/n): ")
                if retry.lower() != 'y':
                    return None, None
        else:
            print(f"无法获取设备 {device_path} 的标识符")
            print("可能的原因:")
            print("- 设备没有正确连接")
            print("- 设备驱动未正确加载")
            print("- 权限不足")
            
            retry = input("是否重新输入设备路径? (y/n): ")
            if retry.lower() != 'y':
                return None, None

def detect_tty_by_plugin(device_name, device_symlink):
    """通过插拔 TTY 设备自动检测获取标识符"""
    print(f"\n自动检测模式 - {device_name}")
    
    input(f"请确保 {device_name} 已断开连接，然后按Enter继续...")
    
    initial_devices = set(os.listdir('/dev'))
    print(f"初始设备列表已记录，现在请插入 {device_name}...")
    
    countdown = 30
    while countdown > 0:
        time.sleep(1)
        countdown -= 1
        sys.stdout.write(f"\r等待设备连接... {countdown}秒 (Ctrl+C取消)")
        sys.stdout.flush()
        
        current_devices = set(os.listdir('/dev'))
        new_devices = current_devices - initial_devices
        
        for new_dev in new_devices:
            if new_dev.startswith('ttyUSB') or new_dev.startswith('ttyACM'):
                device_path = f'/dev/{new_dev}'
                print(f"\n\n检测到新设备: {device_path}")
                
                confirm = input(f"这是您的 {device_name} 设备吗? (y/n): ")
                if confirm.lower() == 'y':
                    descriptor = get_tty_device_descriptor(device_path)
                    if descriptor:
                        return device_symlink, descriptor
                    else:
                        print(f"无法获取设备 {device_path} 的描述符")
                        return None, None
                else:
                    print("继续等待其他设备...")
        
        if countdown % 10 == 0:
            try:
                print("\n")
                check = input("未检测到目标设备，是否继续等待? (y/n/m=切换到手动模式): ")
                if check.lower() == 'n':
                    print("已取消设备检测")
                    return None, None
                elif check.lower() == 'm':
                    print("切换到手动输入模式...")
                    return detect_tty_by_manual_input(device_name, device_symlink)
            except EOFError:
                return None, None
    
    print("\n\n等待超时。")
    switch_mode = input("是否切换到手动输入模式? (y/n): ")
    if switch_mode.lower() == 'y':
        return detect_tty_by_manual_input(device_name, device_symlink)
    
    return None, None

def install_tty_rule():
    """安装 TTY 设备udev规则的交互流程"""
    print_header("TTY 模式 - 安装规则")
    print("添加新 TTY 设备规则 (例如: /dev/ttyUSB* -> /dev/laser)")
    print("-" * 60)
    print("(输入 0 可返回主菜单)")
    print()
    
    device_name = input("请输入设备描述名称 (例如: 激光雷达, 底盘串口): ").strip()
    if not device_name or device_name == "0":
        return
    
    device_symlink = input(f"请输入设备符号链接名 (将作为 /dev/XXX 的XXX部分，如laser, chassis): ").strip()
    if not device_symlink or device_symlink == "0":
        return
    
    if not device_symlink.replace('_', '').isalnum():
        print("符号链接名无效! 只能包含字母、数字和下划线")
        input("按Enter继续...")
        return
    
    output_dir = os.path.join(os.getcwd(), "temp_rules")
    os.makedirs(output_dir, exist_ok=True)
    
    symlink, descriptor = detect_tty_device(device_name, device_symlink)
    if not symlink or not descriptor:
        print("未能成功检测设备或获取描述符")
        input("按Enter继续...")
        return
    
    if create_tty_udev_rule(symlink, descriptor, output_dir):
        print(f"\n已成功为 {device_name} 创建规则")
        install_rule_to_system(symlink, output_dir)
    
    input("\n按Enter返回主菜单...")

def tty_main_menu():
    """TTY 模式的主菜单循环"""
    while True:
        print_header("TTY 模式 - 主菜单")
        print("1. 安装 TTY 设备规则 (ttyUSB* / ttyACM*)")
        print("2. 列出已安装规则")
        print("3. 卸载设备规则")
        print("4. 帮助")
        print("0. 返回上一级菜单")
        
        choice = input("\n请选择操作 [0-4]: ")
        
        if choice == "1":
            install_tty_rule()
        elif choice == "2":
            list_installed_rules()
        elif choice == "3":
            uninstall_device_rule()
        elif choice == "4":
            show_help() # 帮助菜单是通用的
        elif choice == "0":
            break
        else:
            print("无效选项! 请重试")
            input("按Enter继续...")

# ==============================================================================
# CAN 模式 (新增功能)
# ==============================================================================

def get_can_usb_attrs(iface_name):
    """获取CAN接口的idVendor和idProduct"""
    try:
        sys_path = f"/sys/class/net/{iface_name}"
        if not os.path.exists(sys_path):
            print(f"CAN 接口路径不存在: {sys_path}")
            return None, None

        # -a 打印所有属性, -p 指定路径
        result = subprocess.run(
            ["udevadm", "info", "-a", "-p", sys_path],
            capture_output=True, text=True, check=True
        )
        
        vendor_id, product_id = None, None
        
        # 逐行解析 udevadm 的输出
        for line in result.stdout.split('\n'):
            line = line.strip()
            # 我们要查找的是父级USB设备的属性
            if 'ATTRS{idVendor}' in line and not vendor_id:
                vendor_id = line.split('"')[1]
            if 'ATTRS{idProduct}' in line and not product_id:
                product_id = line.split('"')[1]
            
            if vendor_id and product_id:
                return vendor_id, product_id
                
        print(f"在 {iface_name} 的 udev 信息中未找到 idVendor/idProduct")
        print("请确保这是一个 USB-to-CAN 设备。")
        return None, None

    except subprocess.CalledProcessError as e:
        print(f"获取 {iface_name} 设备描述符时出错: {e}")
    return None, None

def create_can_udev_rule(device_name, vendor_id, product_id, output_dir):
    """创建 CAN (SocketCAN) udev规则文件"""
    # 注意: CAN 设备使用 NAME:= 来重命名接口, 而不是 SYMLINK+=
    rule = f'SUBSYSTEM=="net", ACTION=="add", ATTRS{{idVendor}}=="{vendor_id}", ATTRS{{idProduct}}=="{product_id}", NAME:="{device_name}"'
    rule_file = os.path.join(output_dir, f'{device_name}.rules')
    try:
        with open(rule_file, 'w') as f:
            f.write(rule + '\n')
        print(f"已创建 {device_name} 的udev规则: {rule_file}")
        return True
    except IOError as e:
        print(f"写入 {device_name} 的udev规则时出错: {e}")
    return False

def detect_can_device(device_name, device_iface_name):
    """交互式检测 CAN 设备"""
    print(f"\n正在设置 {device_name} (CAN) 的udev规则...")
    print(f"该设备将拥有固定接口名称: {device_iface_name}")
    
    print("\n请选择获取设备标识符的方式:")
    print("1. 自动检测 (插拔设备自动识别)")
    print("2. 手动输入 (直接输入已知的设备接口)")
    print("0. 返回上级菜单")
    
    while True:
        choice = input("\n请选择 [0-2]: ").strip()
        
        if choice == "0":
            return None, None, None
        elif choice == "1":
            return detect_can_by_plugin(device_name, device_iface_name)
        elif choice == "2":
            return detect_can_by_manual_input(device_name, device_iface_name)
        else:
            print("无效选项，请重新选择!")

def detect_can_by_manual_input(device_name, device_iface_name):
    """手动输入 CAN 设备接口获取标识符"""
    print(f"\n手动输入模式 - {device_name}")
    print("提示: 可以使用以下命令查看当前可用的CAN接口:")
    print("  ls /sys/class/net/ | grep can")
    print("  ip link show type can")
    print()
    
    while True:
        iface = input("请输入当前设备接口 (例如: can0) (输入0返回): ").strip()
        
        if not iface:
            print("接口名称不能为空!")
            continue
        
        if iface == "0":
            return None, None, None
        
        device_path = f'/sys/class/net/{iface}'
        if not os.path.exists(device_path):
            print(f"设备接口 {device_path} 不存在!")
            retry = input("是否重新输入? (y/n): ")
            if retry.lower() != 'y':
                return None, None, None
            continue
        
        if not iface.startswith('can'):
            print("警告: 输入的设备似乎不是 CAN 接口")
            confirm = input("是否继续? (y/n): ")
            if confirm.lower() != 'y':
                continue
        
        print(f"\n正在获取接口 {iface} 的USB标识符...")
        vendor_id, product_id = get_can_usb_attrs(iface)
        
        if vendor_id and product_id:
            print(f"成功获取设备标识符: idVendor={vendor_id}, idProduct={product_id}")
            confirm = input(f"确认使用此设备 {iface} 作为 {device_name}? (y/n): ")
            if confirm.lower() == 'y':
                return device_iface_name, vendor_id, product_id
            else:
                retry = input("是否重新输入设备接口? (y/n): ")
                if retry.lower() != 'y':
                    return None, None, None
        else:
            print(f"无法获取设备 {iface} 的USB标识符")
            print("可能的原因:")
            print("- 设备没有正确连接")
            print("- 这不是一个 USB-to-CAN 设备")
            print("- 权限不足")
            
            retry = input("是否重新输入设备接口? (y/n): ")
            if retry.lower() != 'y':
                return None, None, None

def detect_can_by_plugin(device_name, device_iface_name):
    """通过插拔 CAN 设备自动检测获取标识符"""
    print(f"\n自动检测模式 - {device_name}")
    
    input(f"请确保 {device_name} (USB-to-CAN) 已断开连接，然后按Enter继续...")
    
    initial_devices = set(os.listdir('/sys/class/net/'))
    print(f"初始设备列表已记录，现在请插入 {device_name}...")
    
    countdown = 30
    while countdown > 0:
        time.sleep(1)
        countdown -= 1
        sys.stdout.write(f"\r等待设备连接... {countdown}秒 (Ctrl+C取消)")
        sys.stdout.flush()
        
        current_devices = set(os.listdir('/sys/class/net/'))
        new_devices = current_devices - initial_devices
        
        for new_dev in new_devices:
            if new_dev.startswith('can'):
                print(f"\n\n检测到新CAN接口: {new_dev}")
                
                confirm = input(f"这是您的 {device_name} 设备吗? (y/n): ")
                if confirm.lower() == 'y':
                    vendor_id, product_id = get_can_usb_attrs(new_dev)
                    if vendor_id and product_id:
                        print(f"成功获取设备标识符: idVendor={vendor_id}, idProduct={product_id}")
                        return device_iface_name, vendor_id, product_id
                    else:
                        print(f"无法获取设备 {new_dev} 的USB标识符")
                        return None, None, None
                else:
                    print("继续等待其他设备...")
        
        if countdown % 10 == 0:
            try:
                print("\n")
                check = input("未检测到目标设备，是否继续等待? (y/n/m=切换到手动模式): ")
                if check.lower() == 'n':
                    print("已取消设备检测")
                    return None, None, None
                elif check.lower() == 'm':
                    print("切换到手动输入模式...")
                    return detect_can_by_manual_input(device_name, device_iface_name)
            except EOFError:
                return None, None, None
    
    print("\n\n等待超时。")
    switch_mode = input("是否切换到手动输入模式? (y/n): ")
    if switch_mode.lower() == 'y':
        return detect_can_by_manual_input(device_name, device_iface_name)
    
    return None, None, None

def install_can_rule():
    """安装 CAN 设备udev规则的交互流程"""
    print_header("CAN 模式 - 安装规则")
    print("添加新 CAN 设备规则 (例如: can0 -> can_chassis)")
    print("注意: 这适用于 USB-to-CAN 适配器 (SocketCAN)")
    print("-" * 60)
    print("(输入 0 可返回主菜单)")
    print()
    
    device_name = input("请输入设备描述名称 (例如: 底盘CAN, 雷达CAN): ").strip()
    if not device_name or device_name == "0":
        return
    
    device_iface_name = input(f"请输入设备固定接口名 (将作为 canX 的新名称，如 can_chassis): ").strip()
    if not device_iface_name or device_iface_name == "0":
        return
    
    if not device_iface_name.replace('_', '').isalnum():
        print("接口名无效! 只能包含字母、数字和下划线")
        input("按Enter继续...")
        return
    
    output_dir = os.path.join(os.getcwd(), "temp_rules")
    os.makedirs(output_dir, exist_ok=True)
    
    # 检测设备并创建规则
    iface_name, vendor_id, product_id = detect_can_device(device_name, device_iface_name)
    if not iface_name or not vendor_id or not product_id:
        print("未能成功检测设备或获取描述符")
        input("按Enter继续...")
        return
    
    # 创建规则文件
    if create_can_udev_rule(iface_name, vendor_id, product_id, output_dir):
        print(f"\n已成功为 {device_name} 创建规则")
        install_rule_to_system(iface_name, output_dir)
    
    input("\n按Enter返回主菜单...")

def can_main_menu():
    """CAN 模式的主菜单循环"""
    while True:
        print_header("CAN 模式 - 主菜单")
        print("1. 安装 CAN 设备规则 (can* -> can_chassis)")
        print("2. 列出已安装规则")
        print("3. 卸载设备规则")
        print("4. 帮助")
        print("0. 返回上一级菜单")
        
        choice = input("\n请选择操作 [0-4]: ")
        
        if choice == "1":
            install_can_rule()
        elif choice == "2":
            list_installed_rules() # 复用
        elif choice == "3":
            uninstall_device_rule() # 复用
        elif choice == "4":
            show_can_help() # 新的帮助
        elif choice == "0":
            break
        else:
            print("无效选项! 请重试")
            input("按Enter继续...")

# ==============================================================================
# 通用功能 (TTY 和 CAN 共享)
# ==============================================================================

def install_rule_to_system(rule_name, output_dir):
    """(通用) 询问用户是否安装规则到 /etc/udev/rules.d/"""
    install = input("\n是否立即安装此规则到系统? (y/n): ")
    if install.lower() == 'y':
        try:
            rule_file = os.path.join(output_dir, f"{rule_name}.rules")
            dest_file = f"/etc/udev/rules.d/99-{rule_name}.rules" # 添加 99- 前缀
            
            try:
                shutil.copy(rule_file, dest_file)
            except PermissionError:
                print("需要管理员权限安装规则...")
                subprocess.run(["sudo", "cp", rule_file, dest_file], check=True)
            
            print("正在重新加载udev规则...")
            subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
            subprocess.run(["sudo", "udevadm", "trigger"], check=True)
            print(f"\n规则已成功安装! 设备现在将以 {rule_name} 的形式出现")
            print(f"请重新插拔设备以使新规则生效")
        except Exception as e:
            print(f"安装规则时出错: {e}")
            print("\n请手动安装规则:")
            print(f"sudo cp {os.path.join(output_dir, rule_name+'.rules')} /etc/udev/rules.d/99-{rule_name}.rules")
            print("sudo udevadm control --reload-rules")
            print("sudo udevadm trigger")
    else:
        print(f"\n规则文件已创建但未安装: {os.path.join(output_dir, rule_name+'.rules')}")
        print("您可以稍后手动安装此规则")

def read_udev_rules(directory):
    """(通用) 读取目录中的udev规则"""
    rules = {}
    try:
        if os.path.exists(directory):
            for filename in os.listdir(directory):
                if filename.endswith('.rules'):
                    try:
                        with open(os.path.join(directory, filename), 'r') as f:
                            rules[filename] = f.read()
                    except Exception as e:
                        print(f"读取文件 {filename} 时出错: {e}")
    except Exception as e:
        print(f"读取目录 {directory} 时出错: {e}")
    return rules

def list_installed_rules():
    """(通用) 列出已安装的udev规则"""
    print_header("已安装的设备规则")
    
    rules_dir = "/etc/udev/rules.d/"
    rules = read_udev_rules(rules_dir)
    
    if not rules:
        print("未找到任何已安装的规则文件")
    else:
        print(f"在 {rules_dir} 中找到的规则:\n")
        for i, (filename, content) in enumerate(rules.items(), 1):
            print(f"{i}. 文件: {filename}")
            print(f"   规则: {content.strip()}\n")
    
    input("\n按Enter返回主菜单...")

def uninstall_device_rule():
    """(通用) 卸载设备udev规则的交互流程"""
    print_header("卸载设备规则")
    
    rules_dir = "/etc/udev/rules.d/"
    rule_files = []
    
    try:
        if os.path.exists(rules_dir):
            rule_files = [f for f in os.listdir(rules_dir) if f.endswith('.rules')]
    except Exception as e:
        print(f"读取规则目录时出错: {e}")
    
    if not rule_files:
        print("未找到任何已安装的设备规则")
        input("\n按Enter返回主菜单...")
        return
    
    print("选择要卸载的设备规则:\n")
    for i, rule_file in enumerate(rule_files, 1):
        print(f"{i}. {rule_file}")
    
    print("0. 返回主菜单")
    
    choice = input("\n请输入要卸载的设备编号 [0-{}]: ".format(len(rule_files)))
    if choice == "0":
        return
    
    try:
        index = int(choice) - 1
        if 0 <= index < len(rule_files):
            rule_file = rule_files[index]
            rule_path = os.path.join(rules_dir, rule_file)
            
            confirm = input(f"\n确定要卸载 {rule_path} 吗? (y/n): ")
            
            if confirm.lower() == 'y':
                try:
                    try:
                        os.remove(rule_path)
                    except PermissionError:
                        print("需要管理员权限删除规则...")
                        subprocess.run(["sudo", "rm", rule_path], check=True)
                    
                    print("正在重新加载udev规则...")
                    subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
                    subprocess.run(["sudo", "udevadm", "trigger"], check=True)
                    print(f"\n已成功卸载 {rule_file}")
                except Exception as e:
                    print(f"卸载规则时出错: {e}")
                    print("请尝试手动删除规则文件:")
                    print(f"sudo rm {rule_path}")
                    print("sudo udevadm control --reload-rules")
            else:
                print("已取消卸载操作")
        else:
            print("无效的选项!")
    except ValueError:
        print("请输入有效的数字!")
    
    input("\n按Enter返回主菜单...")

def show_help():
    """(通用) 显示TTY帮助信息"""
    print_header("TTY 模式 - 帮助")
    print("使用指南:\n")
    print("1. 安装设备规则: 为 TTY 设备 (ttyUSB*, ttyACM*) 创建 udev 规则")
    print("   - 规则安装后，设备将始终使用固定的符号链接 (SYMLINK)\n")
    
    print("2. 什么是符号链接 (SYMLINK)?")
    print("   - 这是一个永久性的“快捷方式”")
    print("   - 示例: /dev/ttyUSB0 -> /dev/laser")
    print("   - 即使设备插到不同的USB口，/dev/laser 始终指向它\n")
    
    print("3. 使用固定设备名的好处:")
    print("   - 避免设备重启后端口名称变化 (ttyUSB0 变为 ttyUSB1)")
    print("   - 使launch文件配置更简单（port:=/dev/laser）\n")
    
    print("4. 常见设备类型示例:")
    print("   - 激光雷达: /dev/laser")
    print("   - 底盘串口: /dev/chassis")
    print("   - IMU传感器: /dev/imu")
    print("   - GPS模块: /dev/gps\n")
    
    print("提示: 可能需要管理员权限来安装或卸载规则")
    
    input("\n按Enter返回主菜单...")

def show_can_help():
    """(新增) 显示CAN帮助信息"""
    print_header("CAN 模式 - 帮助")
    print("使用指南:\n")
    print("1. 安装设备规则: 为 USB-to-CAN 适配器 (SocketCAN) 创建 udev 规则")
    print("   - 这适用于使用 gs_usb, kvaser, peak_usb 等驱动的设备")
    print("   - 规则安装后，设备将始终使用固定的网络接口名称 (NAME)\n")
    
    print("2. 什么是接口重命名 (NAME:=)?")
    print("   - 这会强制重命名内核网络接口")
    print("   - 示例: can0 -> can_chassis")
    print("   - 无论您插入多少个CAN设备，您的底盘CAN总线始终叫 can_chassis\n")
    
    print("3. 使用固定设备名的好处:")
    print("   - 避免设备重启后接口名称变化 (can0 变为 can1)")
    print("   - 使launch文件配置更简单（iface:=can_chassis）\n")
    
    print("4. 常见设备类型示例:")
    print("   - 底盘CAN: can_chassis")
    print("   - 机械臂CAN: can_arm\n")
    
    print("提示: 可能需要管理员权限来安装或卸载规则")
    
    input("\n按Enter返回主菜单...")

# ==============================================================================
# 主程序入口
# ==============================================================================

def main():
    """新的顶层主菜单，用于选择模式"""
    if os.name == 'nt':
        print("错误: 此脚本专为 Linux (udev) 系统设计。")
        print("在 Windows 上无效。")
        input("按Enter退出...")
        return
    
    if os.geteuid() == 0:
        print("警告: 建议不要以 root 身份直接运行此脚本。")
        print("脚本会在需要时通过 'sudo' 请求管理员权限。")
        input("按Enter继续 (不推荐)...")

    while True:
        clear_screen()
        print("=" * 60)
        print("              ROS2 设备 UDEV 规则管理器")
        print("=" * 60)
        print("请选择要管理的设备类型:")
        print("1. TTY 串口设备 (如: ttyUSB*, ttyACM*)")
        print("2. CAN 网络接口 (如: can*)")
        print("0. 退出程序")
        
        choice = input("\n请选择模式 [0-2]: ").strip()
        
        if choice == "1":
            tty_main_menu()
        elif choice == "2":
            can_main_menu()
        elif choice == "0":
            print("\n谢谢使用，再见!")
            break
        else:
            print("无效选项! 请重试")
            input("按Enter继续...")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
        print("正在退出...")
    except Exception as e:
        print(f"\n程序发生错误: {e}")
        print("请尝试重新运行程序")
    finally:
        # 清理临时目录
        output_dir = os.path.join(os.getcwd(), "temp_rules")
        if os.path.exists(output_dir):
            try:
                shutil.rmtree(output_dir)
                print(f"\n已清理临时目录: {output_dir}")
            except Exception:
                pass # 忽略清理失败
        
        print("再见!")