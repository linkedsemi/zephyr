# coding=utf-8

import configparser
import logging
import os
import time
import subprocess
import sys


if getattr(sys, 'frozen', False):
    # run executable
    base_path = os.path.dirname(sys.executable)
else:
    # run .py
    base_path = os.path.abspath(os.path.dirname(__file__))


config_file = os.path.join(base_path, 'arottool_config.ini')


arot_provision_gen_dir = os.path.join(base_path, "arot_provision_gen")
provision_gen_dir = os.path.join(arot_provision_gen_dir, "provision_gen")
provision_json_dir = os.path.join(arot_provision_gen_dir, "json")
provision_bin_dir = os.path.join(arot_provision_gen_dir, "bin")
pack_path = os.path.join(provision_gen_dir, "pack.py")


uart = None
DEBUG = 0


def read_file(filename, opt=None):
    """
    读取文件
    :param filename: 文件名
    :param opt: 执行操作，如设置FRU
    """
    try:
        with open(filename, "rb") as file:
            read_data = file.read()
            file_size = len(read_data)
            if opt == "FRU":
                if file_size == 0 or file_size > 2 * 1024:
                    print("File size is illegal, should be 0 < size < 2 KB ")
                    return -1
        print(f"Image size: {file_size} bytes")
        block_data = []
        for i in range(0, file_size, 128):
            block = read_data[i:i + 128]
            if len(block) < 128:
                block += bytes(128 - len(block))
            block_data.append(block)
        return block_data
    except IOError as e:  # 捕获文件操作异常
        print(f"Failed to open file: {filename}, {e}")
        return -1


def signed_bin_file_gen(config_opt):
    """
    生成签名文件
    :param config_opt: ARoT Configuration(provision,feature,pubkey)
    :return: 返回执行状态
    """
    opt = ["provision", "pubkey", "feature"]
    if config_opt not in opt:
        print("[Error] Invalid Configuration Option")
        return -1

    json_file = os.path.join(provision_json_dir, f"{config_opt}.json")
    if not os.path.exists(provision_bin_dir):
        os.mkdir(provision_bin_dir)
    command = ["python3", pack_path, config_opt, json_file]
    if config_opt == "pubkey":
        key_pem_file = os.path.join(provision_json_dir, "ecdsa_public_key.pem")
        command = ["python3", pack_path, config_opt, key_pem_file]
    try:
        subprocess.run(command, check=True, text=True)
        time.sleep(0.5)
        if os.path.exists(os.path.join(provision_bin_dir, f"{config_opt}.bin")):
            print("[Success] Signed Binary File Generated Successfully")
            ret = 0
            return ret
    except subprocess.CalledProcessError as e:
        print(f"Command execution failed with exit code {e.returncode}.\n Error message: \n{e.stderr}")
        ret = -1
        return ret


def read_generated_bin_file(file):
    """
    读取生成的签名文件
    :param file: file name
    :return: 返回执行状态
    """
    if os.path.exists(file):
        try:
            with open(file, "rb") as file:
                data = file.read()
                return data
        except IOError as e:
            print(f"Failed to open file: {file}, Error is {e}")
    else:
        print("[Error] Signed Binary File Not Found, Please Generate first")
        return -1


def clean_log(file):
    """
    清除日志文件
    :param file: 日志文件
    """
    if os.path.exists(file):
        os.remove(file)


def setup_logger(name, log_dir='log', enable_logging=False):
    logger = None
    if enable_logging:
        # 创建日志目录（如果不存在）
        if not os.path.exists(log_dir):
            os.mkdir(log_dir)

        # 获取当前时间并格式化为文件名
        log_file = os.path.join(log_dir, f'{name}')

        # 创建日志配置
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)

        # 创建文件处理器
        file_handler = logging.FileHandler(log_file, mode='a', encoding='utf-8')
        file_handler.setLevel(logging.INFO)

        # 创建终端处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # 添加处理器到日志记录器
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

    return logger, file_handler


def log_message(logger, message, level='info', enable_formatter=True):
    if logger:
        if enable_formatter:
            log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            for handler in logger.handlers:
                handler.setFormatter(log_formatter)
        else:
            for handler in logger.handlers:
                handler.setFormatter(logging.Formatter(''))

        if level == 'error':
            logger.error(message)
        elif level == 'warning':
            logger.warning(message)
        elif level == 'debug':
            logger.debug(message)
        else:
            logger.info(message)


def read_config(header, option):
    """
    Get Version Information
    """
    config = configparser.ConfigParser()
    try:
        config.read(config_file)

        if config.has_option(header, option):
            return config.get(header, option)
        else:
            raise ValueError(f'Option "{option}" not found in {config_file}')

    except ValueError as e:
        logging.error(f"Error : {e}")
    except Exception as e:
        logging.error(f"Error : {e}")
        raise ValueError