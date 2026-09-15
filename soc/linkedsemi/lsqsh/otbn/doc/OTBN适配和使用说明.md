# OTBN 使用说明

> **适用对象**: LinkedSemi QSH 系列芯片
> **文档版本**: v2.0
> **核心头文件**: `zephyr/soc/linkedsemi/lsqsh/otbn/ls_otbn_config.h`

---

## 1. 概述

OTBN（OpenTitan Big Number Accelerator）是 LinkedSemi QSH 系列芯片中的硬件协处理器，用于加速大数运算和密码学操作（如 ECC、RSA、SHA、SM3 等）。

### 1.1 OTBN 与主处理器的交互（摘自 OpenTitan 官方文档，已适配本芯片）

> 来源：https://opentitan.org/book/hw/ip/otbn/doc/otbn_intro.html

OTBN 拥有独立的指令存储器（IMem）和数据存储器（DMem），与主处理器 ls_qsh 的存储器完全隔离。要运行一个 OTBN 程序，ls_qsh 将程序加载到 OTBN 的指令存储器中（如果程序包含预置常量，还需加载到数据存储器），然后将输入数据写入数据存储器，并触发"执行"命令。当 OTBN 执行完成后，向 ls_qsh 发送中断，ls_qsh 从数据存储器中读回结果。

OTBN 通过以下机制保护其数据不被 ls_qsh 访问：

- ls_qsh **不能**在 OTBN 正在执行程序时读取 OTBN 的存储器。
- ls_qsh **不能**在 OTBN 执行中途将其停止，必须等待 OTBN 执行完成。
- Key Manager 可以将密钥**直接旁路注入** OTBN，无需让 ls_qsh 获得密钥访问权限。
- 如果 OTBN 遇到错误，它会**自行锁定**，这意味着 OTBN 完成后 ls_qsh 无法读回任何程序数据（只能从特殊寄存器读取一个错误码）。

这些机制共同确保 OTBN 与 ls_qsh 之间保持足够的隔离，使其成为一个**安全边界**。

### 1.2 本驱动的 Session 模型

OTBN 硬件同一时间只能执行一个任务。本层封装提供了一套基于 **Session（会话）** 的独占访问模型，确保多线程环境下对 OTBN 的安全使用。

所有 OTBN 操作必须遵循 **acquire → load firmware / write data → execute → read result → release** 的顺序，详见后续章节。

---

## 2. 核心接口

所有用户层 API 定义在：

```raw
zephyr/soc/linkedsemi/lsqsh/otbn/ls_otbn_config.h
```

### 2.1 Session 管理接口

| 接口 | 使用说明 |
|------|----------|
| `ls_otbn_session_acquire(firmware_id, timeout)` | 获取 OTBN 独占会话。`firmware_id` 指定本次要使用的固件；`timeout` 单位为秒，`0` 非阻塞，`N` 最多等待 N 秒。成功返回 `0`；若 OTBN 已被占用或超时，返回 `-EBUSY`。 |
| `ls_otbn_session_release()` | 释放当前线程持有的 OTBN 会话。必须由 acquire 成功的同一线程调用，否则会触发断言。 |
| `ls_otbn_session_is_owner()` | 检查当前线程是否持有 OTBN 会话。主要用于 hash 等分段式操作，在多次调用之间确认 still owner。返回 `true` / `false`。 |
| `ls_otbn_module_init()` | 初始化 OTBN 硬件及 OS 资源（信号量、互斥锁、中断）。幂等，通常由 `ls_otbn_session_acquire` 自动调用，无需用户手动调用。 |
| `ls_otbn_module_deinit()` | 反初始化 OTBN。会等待当前会话释放、等待 OTBN 执行完成后，再清理硬件和资源。返回 `0` 或负 errno。 |

### 2.2 命令与内存访问接口

| 接口 | 使用说明 |
|------|----------|
| `ls_otbn_cmd(cmd)` | 向 OTBN 发送命令并阻塞等待完成。`cmd` 可以是 `OTBN_CMD_EXECUTE`、`OTBN_CMD_SEC_WIPE_DMEM` / `OTBN_CMD_SEC_WIPE_IMEM`。发送命令前会先清除历史中断标志并 reset 完成信号量，避免脏状态。 |
| `ls_otbn_imem_write(offset, src, size)` | 写 OTBN 指令内存。`offset` 和 `size` 必须 4 字节对齐。调用前要求当前线程持有 session 且 OTBN 处于 idle，否则返回 `-EACCES` / `-EBUSY`。 |
| `ls_otbn_imem_read(offset, dst, size)` | 读 OTBN 指令内存。对齐、session、idle 检查同上。 |
| `ls_otbn_dmem_write(offset, src, size)` | 写 OTBN 数据内存。对齐、session、idle 检查同上。 |
| `ls_otbn_dmem_read(offset, dst, size)` | 读 OTBN 数据内存。对齐、session、idle 检查同上。 |
| `ls_otbn_dmem_set(offset, data, size)` | 将 DMEM 某区域设置为固定 32 位值。常用于初始化前清零 DMEM。对齐、session、idle 检查同上。 |

### 2.3 回调注册接口

| 接口 | 使用说明 |
|------|----------|
| `ls_otbn_random_callback_register(trng_cb, prng_cb)` | 注册 TRNG/PRNG 回调，供 OTBN 运算过程中请求随机数时使用。`trng_cb` 用于真随机数，`prng_cb` 用于伪随机数；传 `NULL` 则使用内部 fallback（`rand()`，生产环境不建议）。 |
| `ls_otbn_done_callback_register(handler, param)` | 注册 OTBN 执行完成回调。当 OTBN 执行完命令后，中断上下文中会调用 `handler(param)`。 |
| `ls_otbn_done_callback_unregister()` | 注销完成回调。 |

---

## 3. 基本使用流程

所有 OTBN 操作必须遵循 **acquire → use → release** 的顺序：

```c
#include "ls_otbn_config.h"

int run_otbn_operation(void)
{
    int ret;

    /* 1. 获取 OTBN 会话
     *    timeout 单位为秒。
     *    timeout = 0  表示非阻塞，如果 OTBN 正忙立即返回 -EBUSY。
     *    timeout = N  表示最多等待 N 秒，超时返回 -EBUSY。
     */
    ret = ls_otbn_session_acquire(OTBN_FIRMWARE_SHA256, 10);
    if (ret != 0) {
        /* OTBN 正被其他线程使用，或超时 */
        return ret;
    }

    /* 2. 加载固件、写入输入数据 */
    ls_otbn_imem_write(0, (uint32_t *)sha256_firmware, sha256_size);
    ls_otbn_dmem_write(0x140, (uint32_t *)msg, msg_len);

    /* 3. 执行 OTBN 命令 */
    ret = ls_otbn_cmd(OTBN_CMD_EXECUTE);
    if (ret != 0) {
        ls_otbn_session_release();
        return ret;
    }

    /* 4. 读取结果 */
    ls_otbn_dmem_read(0x0, (uint32_t *)result, result_size);

    /* 5. 释放 OTBN 会话 */
    ls_otbn_session_release();
    return 0;
}
```

### 3.1 关键规则

- **独占性**：同一时间只有一个线程能持有 OTBN 会话。
- **timeout 单位**：`ls_otbn_session_acquire` 的 `timeout` 参数单位为秒；`0` 表示非阻塞，`N` 表示最多等待 N 秒。
- **成对使用**：`ls_otbn_session_acquire` 和 `ls_otbn_session_release` 必须成对出现。
- **线程绑定**：只有 acquire 成功的线程才能调用 `release`。
- **不可重入**：同一线程在未 release 前再次 acquire 会返回 `-EBUSY`。
- **内存访问权限**：`ls_otbn_imem_xxx` / `ls_otbn_dmem_xxx` 要求当前线程持有会话，否则返回 `-EACCES`。
- **对齐要求**：`offset` 和 `size` 必须 4 字节对齐，否则返回 `-EINVAL`。
- **空闲检查**：内存访问接口会检查 OTBN 是否处于 idle 状态，若正在执行命令则返回 `-EBUSY`。

---

## 4. 分段式操作（以 Hash 为例）

对于需要分多次调用的算法（如 SHA256 的 init/update/final），可以在多次调用之间保持会话：

```c
static otbn_firmware_t g_hash_firmware;

int hash_init(otbn_firmware_t firmware_id)
{
    int ret = ls_otbn_session_acquire(firmware_id, 10);
    if (ret != 0) {
        return ret;
    }
    g_hash_firmware = firmware_id;

    /* 加载 hash 固件，初始化 DMEM */
    ls_otbn_imem_write(0, (uint32_t *)hash_imem, hash_imem_size);
    ls_otbn_dmem_set(0, 0, OTBN_DMEM_SIZE);

    return 0;
}

int hash_update(uint8_t *msg, uint32_t len)
{
    /* 在分段调用中验证当前线程仍持有会话 */
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }

    /* 写入数据并执行 */
    ls_otbn_dmem_write(0x140, (uint32_t *)msg, len);
    ls_otbn_cmd(OTBN_CMD_EXECUTE);

    return 0;
}

int hash_final(uint8_t *result)
{
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }

    /* 填充 padding，执行 final，读取结果 */
    ls_otbn_dmem_write(0x140, (uint32_t *)padding, padding_len);
    ls_otbn_cmd(OTBN_CMD_EXECUTE);
    ls_otbn_dmem_read(0x0, (uint32_t *)result, 32);

    ls_otbn_session_release();
    return 0;
}
```

---

## 5. 固件 ID

OTBN 通过固件 ID 区分不同的算法镜像：

```c
#define OTBN_FIRMWARE_UNUSED         0
#define OTBN_FIRMWARE_SHA256         1
#define OTBN_FIRMWARE_SHA384         2
#define OTBN_FIRMWARE_SHA512         3
#define OTBN_FIRMWARE_SM3            4
#define OTBN_FIRMWARE_ECDSA_P256     5
#define OTBN_FIRMWARE_ECDSA_P384     6
#define OTBN_FIRMWARE_SM2            7

#define OTBN_FIRMWARE_USER_BASE     0x100
```

用户自定义固件可以从 `OTBN_FIRMWARE_USER_BASE` 开始分配 ID：

```c
#define OTBN_FIRMWARE_MY_ALGO    (OTBN_FIRMWARE_USER_BASE + 0)
```

---

## 6. 加密库适配层

### 6.1 mbedtls 适配

mbedtls 的 OTBN 替代实现位于：

```raw
zephyr/soc/linkedsemi/lsqsh/otbn/mbedtls/
modules/crypto/mbedtls/library/*_alt.c
```

mbedtls 适配层内部封装了 `ls_otbn_session_acquire/release`，用户无需手动管理。不同算法的 session 生命周期不同：

#### 6.1.1 Hash 类算法（SHA256 / SHA384 / SHA512 / SM3）

Hash 是分段式操作，session 在 `starts` 时获取，在 `finish` 时释放：

```raw
mbedtls_sha256_starts()
    └── mbedtls_ls_otbn_operation_init(OTBN_FIRMWARE_SHA256)
            └── ls_otbn_session_acquire(OTBN_FIRMWARE_SHA256, 10)

mbedtls_sha256_update()
    └── ls_otbn_session_is_owner()   ← 分段调用中验证 still owner
    └── ls_otbn_imem_write() / ls_otbn_dmem_write() / ls_otbn_cmd()

mbedtls_sha256_finish()
    └── ls_otbn_dmem_read()
    └── ls_otbn_session_release()
```

#### 6.1.2 ECDSA 算法

ECDSA 是一次性操作，session 在 sign/verify/genkey 开始时获取，结束时释放：

```raw
mbedtls_ecdsa_sign()
    └── mbedtls_ls_otbn_operation_init(OTBN_FIRMWARE_ECDSA_P256)
            └── ls_otbn_session_acquire(OTBN_FIRMWARE_ECDSA_P256, 10)
    └── ls_otbn_imem_write() / ls_otbn_dmem_write() / ls_otbn_cmd()
    └── ls_otbn_dmem_read()
    └── ls_otbn_session_release()
```

### 6.2 wolfssl 适配

wolfssl 的 OTBN 替代实现位于：

```raw
modules/lib/wolfssl/wolfcrypt/src/port/linkedsemi/
```

wolfssl 适配通常在单个函数内完成完整的 acquire → use → release 流程：

```raw
wc_ecc_sign_hash()
    └── ls_otbn_session_acquire(OTBN_FIRMWARE_ECDSA_P256, 10)
    └── ls_otbn_imem_write() / ls_otbn_dmem_write() / ls_otbn_cmd()
    └── ls_otbn_dmem_read()
    └── ls_otbn_session_release()
```

> **注意**：mbedtls 和 wolfssl 的 OTBN 适配不能同时使用，二者会竞争同一个 OTBN 硬件 session。

---

## 7. 新增一个 OTBN 固件

### 7.1 准备固件数据

OTBN 固件的原始形式是由 OpenTitan 工具链编译生成的 `.elf` 文件。在该 ELF 文件中：

- **`.text` 段** → 对应 OTBN 的 **IMEM**（指令存储器）
- **`.data` 段** → 对应 OTBN 的 **DMEM**（数据存储器）

提取步骤：

1. 使用 OpenTitan 工具链编译算法源码，生成 `xxx.elf`
2. 从 ELF 中提取 `.text` 和 `.data` 段的原始字节数据
3. 将提取的数据转换为 C 语言数组

示例：

```c
// my_algo_text.c
const uint8_t my_algo_imem[] = { 0x37, 0x01, 0x00, 0x00, ... };  // IMEM 指令数据
const uint8_t my_algo_dmem[] = { ... };                           // DMEM 初始数据
```

### 7.2 获取 DMEM 偏移

固件编译后会生成反汇编文件，其中变量通常以 `_otbn_local_app_xxx` 格式命名。这些符号地址就是变量在 DMEM 中的偏移，上层通过 `ls_otbn_dmem_write()` / `ls_otbn_dmem_read()` 读写这些地址。

例如：

```c
#define MY_ALGO_INPUT_OFFSET   0x100   // 来自反汇编符号 _otbn_local_app_my_algo_input
#define MY_ALGO_OUTPUT_OFFSET  0x200   // 来自反汇编符号 _otbn_local_app_my_algo_output
```

### 7.3 定义固件 ID

用户自定义固件必须从 `OTBN_FIRMWARE_USER_BASE`（值为 `0x100`）开始向上分配 ID，避免与内置固件 ID 冲突。

```c
/* 用户自定义固件 ID，必须 >= OTBN_FIRMWARE_USER_BASE */
#define OTBN_FIRMWARE_MY_ALGO    (OTBN_FIRMWARE_USER_BASE + 0)
```

### 7.4 在应用中使用

```c
ret = ls_otbn_session_acquire(OTBN_FIRMWARE_MY_ALGO, 10);
if (ret != 0) return ret;

ls_otbn_imem_write(0, (uint32_t *)my_algo_imem, sizeof(my_algo_imem));
ls_otbn_dmem_write(0, (uint32_t *)my_algo_dmem, sizeof(my_algo_dmem));

/* 写入用户输入 */
ls_otbn_dmem_write(0x100, (uint32_t *)input, sizeof(input));

/* 执行 */
ret = ls_otbn_cmd(OTBN_CMD_EXECUTE);
if (ret != 0) {
    ls_otbn_session_release();
    return ret;
}

/* 读取结果 */
ls_otbn_dmem_read(0x200, (uint32_t *)output, sizeof(output));

ls_otbn_session_release();
```

---

## 8. 注意事项

1. **会话必须成对使用**：acquire 成功后，无论后续操作是否成功，都要保证最终调用 release。
2. **不要在持有会话时长时间阻塞**：OTBN 是共享资源，持有会话期间其他线程无法使用。
3. **随机数回调**：生产环境应通过 `ls_otbn_random_callback_register()` 注册硬件 TRNG，避免使用默认的 `rand()`。
4. **完成回调**：如果需要异步通知，可通过 `ls_otbn_done_callback_register()` 注册中断回调。
5. **多核缓存一致性**：在多核系统中，确保 CPU 缓存与 OTBN 内存之间的一致性（必要时执行 cache flush/invalidate）。

---

## 附录：关键文件路径

| 功能 | 路径 |
|------|------|
| 用户 API 头文件 | `zephyr/soc/linkedsemi/lsqsh/otbn/ls_otbn_config.h` |
| Session 与中断管理 | `zephyr/soc/linkedsemi/lsqsh/otbn/otbn_common.c` |
| mbedtls 适配层 | `zephyr/soc/linkedsemi/lsqsh/otbn/mbedtls/` |
| wolfssl 适配层 | `modules/lib/wolfssl/wolfcrypt/src/port/linkedsemi/` |
