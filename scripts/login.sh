#!/bin/bash

# 检查 sshpass 是否安装
if ! command -v sshpass &> /dev/null; then
    echo "Error: sshpass is NOT installed!"
    echo "Please install it first using command:"
    echo "sudo update && sudo apt install sshpass"
    exit 1
fi

# 跳板机 B 的信息
B_USR="sti"
# B_IP="10.30.55.83"
B_IP="10.30.60.215"	# adolph PC IP
B_PWD="Sti@123"	# adolph 密码
TMP_DIR="/home/$B_USR/tmp/"

# 目标机器 C 的信息
C_USR="idc"
C_IP="192.168.195.3"
C_PWD="IDC123@byd"
DIST_DIR="/home/$C_USR/workset/old_lidar_sdk/"

function LogPrint() {
	if [ "$#" -ne 0 ] && [ "$1" = 'error' ]; then
		echo 'The parameters of this script is ERROR!'
	fi

	echo 'Usage: bash login.sh [OPTION...] [FILE]...'
    echo 'Examples:'
    echo '  bash login.sh           #   login into the DC;'
    echo '  bash login.sh -h        # --help  give this help list'
    echo '  bash login.sh -d FILE   # --download  get the FILE from DC'
    echo '  bash login.sh -d FOLDER   # --download  get the FOLDER from DC'
    echo '  bash login.sh FILE      #   send the FILE to DC'
    echo '  bash login.sh FOLDER    #   send the folder to DC'
	exit;
}

function Login() {
    sshpass -p "$B_PWD" ssh -tt "$B_USR@$B_IP" "
        sshpass -p '$C_PWD' ssh '$C_USR@$C_IP'
    "
}

function SCP() {
    if [ $# -lt 1 ]; then
        LogPrint 'SCP() error'
    fi

    # if [ "$1" = "-h" -o "$1" = "--help" ]; then
    if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
        LogPrint
    fi

    if [ "$1" = "-d" ] || [ "$1" == "--download" ]; then
        echo "downloading ..."
        for SRC in "${@:2:$#}"; do
            echo "downloading $DIST_DIR$SRC  to $TMP_DIR ..."
            # 下载目标文件到跳板机器 B 的 temp 目录
            sshpass -p "$B_PWD" ssh "$B_USR@$B_IP" "
                sshpass -p '$C_PWD' scp -r '$C_USR@$C_IP:$DIST_DIR$SRC' '$TMP_DIR'
            "
            # 下载目标文件到本地当前目录
            sshpass -p "$B_PWD" scp -r "$B_USR"@"$B_IP":"$TMP_DIR$SRC" ./
            echo "$SRC is downloaded in $PWD"
        done
    else
        for SRC in "${@:1:$#}"; do
            if [ -d "$SRC" ] || [ -f "$SRC" ]; then
                echo "sending $SRC ..."
                sshpass -p "$B_PWD" scp -r "$SRC" "$B_USR"@"$B_IP":"$TMP_DIR"

                NAME=$(basename "$SRC")

                # 传输文件到目标机器 C 的 Downloads 目录
                sshpass -p "$B_PWD" ssh "$B_USR@$B_IP" "
                    sshpass -p '$C_PWD' scp -r '$TMP_DIR''$NAME' '$C_USR@$C_IP:$DIST_DIR'
                "
            else
                sshpass -p "$B_PWD" ssh "$B_USR@$B_IP" "rm -rf $TMP_DIR/*"
                LogPrint 'error'
            fi
        done
    fi

    sshpass -p "$B_PWD" ssh "$B_USR@$B_IP" "rm -rf $TMP_DIR/*"
}

#    # 文件路径
#    FILE_OR_FOLDER_PATH="$1"
#
#    # 传输文件到目标机器 C 的 Downloads 目录
#    sshpass -p "$B_PWD" ssh "$B_USR@$B_IP" "
#        sshpass -p '$C_PWD' scp -3 '$FILE_OR_FOLDER_PATH' '$C_USR@$C_IP:$DIST_DIR'
#    "

# 检查参数
if [ $# -lt 1 ]; then
    echo "login into the Domain Controller ..."
    # 登录到跳板机 B
    Login
else
    SCP "$@"
fi
