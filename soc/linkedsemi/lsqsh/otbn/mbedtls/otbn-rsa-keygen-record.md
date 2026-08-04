# OTBN RSA keygen 适配记录文档

> 关联otbn仓库 commit节点：a3667aa5f1e6d770393801981683e675a255d489

> 记录当前 Zephyr/mbedtls 中 OTBN RSA keygen 的实现细节、性能数据与后续优化方向。

---

## 1. 总体架构

```raw
应用层（samples/mbedtls_throughput、samples/mbedtls_test）
        │
        ▼
mbedtls_rsa_gen_key()
        │
        ├─── 若 CONFIG_MBEDTLS_RSA_LINKEDSEMI_OTBN_ALT=y
        │     且满足条件（2048/3072/4096、公钥指数 F4）
        │     调用 mbedtls_rsa_gen_key_otbn()
        │           │
        │           ▼
        │     CPU 生成 candidate
        │     CPU trial division（小素数试除）
        │           │
        │           ▼
        │     OTBN CHECK_PRIME（Miller-Rabin，RND base）
        │           │
        │           ▼
        │     OTBN KEY_FROM_PQ（计算 n、d，无 RND）
        │           │
        │           ▼
        │     CPU 推导 CRT 参数并校验私钥
        │
        └─── 否则回退到 mbedtls 软件实现
```

---

## 2. 软件与 OTBN 的职责划分

| 步骤 | 负责方 | 说明 |
|------|--------|------|
| 生成素数候选值 | CPU | 调用 `f_rng` 填充 `prime_bytes`，强制奇数并设置最高两位 |
| 小素数试除 | CPU | 用前 1228 个素数（到 9973）过滤约 80% 合数 |
| Miller-Rabin 素性检测 | OTBN | `CHECK_PRIME` 模式，每轮从 OTBN RND/URND 取随机 base |
| 由 p/q 推导 n、d | OTBN | `KEY_FROM_PQ` 模式，只做模乘和 modinv_f4，不读随机数 |
| CRT 参数推导 | CPU | `mbedtls_rsa_deduce_crt()`（若未启用 `MBEDTLS_RSA_NO_CRT`） |
| 私钥校验 | CPU | `mbedtls_rsa_check_privkey()` |

---

## 3. OTBN 固件模式与 DMEM 布局

### 3.1 模式常量

定义位置：`modules/hal/linkedsemi/hal_driver/inc/ls_otbn_rsa.h`

| 模式 | 值 | 用途 |
|------|-----|------|
| `MODE_RSA_KEYGEN_CHECK_PRIME_2048` | 0x48e | 对 2048-bit 候选值做 Miller-Rabin |
| `MODE_RSA_KEYGEN_CHECK_PRIME_3072` | 0x018 | 对 3072-bit 候选值做 Miller-Rabin |
| `MODE_RSA_KEYGEN_CHECK_PRIME_4096` | 0x442 | 对 4096-bit 候选值做 Miller-Rabin |
| `MODE_RSA_KEYGEN_KEY_FROM_PQ_2048` | 0x3aa | 由 p/q 推导 n、d（2048-bit） |
| `MODE_RSA_KEYGEN_KEY_FROM_PQ_3072` | 0x005 | 由 p/q 推导 n、d（3072-bit） |
| `MODE_RSA_KEYGEN_KEY_FROM_PQ_4096` | 0x3fd | 由 p/q 推导 n、d（4096-bit） |

> 旧的 `MODE_RSA_KEYGEN_*`（GEN 模式）保留在头文件中但已不再使用。

### 3.2 DMEM 布局

`RSA_KEYGEN_DMEM_SIZE = 0x760`

| 偏移 | 名称 | 用途 |
|------|------|------|
| `0x000` | `RSA_KEYGEN_OFFSET_N` / `RSA_KEYGEN_OFFSET_P` | CHECK_PRIME 时写入候选值；KEY_FROM_PQ 时写入 p |
| `0x200` | `RSA_KEYGEN_OFFSET_D` / `RSA_KEYGEN_OFFSET_Q` | CHECK_PRIME 时未使用；KEY_FROM_PQ 时写入 q |
| `0x400` | `RSA_KEYGEN_OFFSET_COFACTOR` | KEY_FROM_PQ 时使用的 cofactor 区域 |
| `0x720` | `RSA_KEYGEN_OFFSET_MODE` | 写入运行模式 |
| `0x740` | `RSA_KEYGEN_OFFSET_STATUS` | CHECK_PRIME 执行后读取结果：`0xFFFFFFFF` 表示通过 |

### 3.3 固件数组

- 文件：`modules/hal/linkedsemi/hal_driver/src/otbn/text_array/rsa_text.c`
- 数组：`const uint8_t rsa_keygen_imem[]`
- 来源：由 `./rsa_keygen/rsa_keygen_firmware.h` 中的方案 B 固件替换而来。
- 大小：**4156 bytes**。

---

## 4. 代码改动清单

| 文件 | 改动内容 |
|------|----------|
| `modules/hal/linkedsemi/hal_driver/src/otbn/text_array/rsa_text.c` | 替换为方案 B keygen 固件数组 |
| `modules/hal/linkedsemi/hal_driver/inc/ls_otbn_rsa.h` | 新增 CHECK_PRIME / KEY_FROM_PQ 模式与偏移常量，更新 DMEM 大小为 `0x760` |
| `modules/crypto/mbedtls/library/rsa_alt.c` | 重新实现 `mbedtls_rsa_gen_key_otbn()`，新增 candidate 生成、小素数试除、CHECK_PRIME / KEY_FROM_PQ 调用、CRT 推导与校验 |
| `linkedsemi_zephyr_project/samples/mbedtls_throughput/src/main.c` | 新增 `rsa_keygen_get_rng()` 使用 TRNG 取熵；keygen 调用使用专用 RNG；保留 `rsa_get_rng()` 用于 padding |
| `linkedsemi_zephyr_project/samples/mbedtls_test/src/main.c` | 新增 `rsa_keygen_get_rng()`；新增 `test_rsa_keygen_size()`  helper，对 2048/3072/4096 分别做 keygen + check_privkey + sign/verify；保留 `test_rng()` 用于 padding/sign/verify |
| `linkedsemi_zephyr_project/samples/mbedtls_test/boards/lsqsh_evb_cpu1.overlay` | 增加 `zephyr,entropy = &trng1;` 并启用 `trng1` |
| `linkedsemi_zephyr_project/samples/mbedtls_test/prj.conf` | 增加 `CONFIG_ENTROPY_GENERATOR=y` |
| `.claude/plans/bubbly-seeking-sutton.md` | 跟踪各 Phase 状态与上板测试数据 |

---

## 5. 关键函数与数据流

### 5.1 `mbedtls_rsa_gen_key_otbn()` 流程

位置：`modules/crypto/mbedtls/library/rsa_alt.c`

1. 校验 `nbits` 为 2048/3072/4096，`exponent` 为 65537；否则返回 `-ENOTSUP`，触发软件回退。
2. 获取 OTBN session（`OTBN_FIRMWARE_RSA_KEYGEN`，重试 10 次）。
3. 写入 keygen IMEM，清空 DMEM。
4. **找素数 P**：
   - `rsa_otbn_gen_candidate()` 生成候选值（奇数、最高两位置 1）。
   - `rsa_otbn_trial_divide()` 用小素数表过滤。
   - `rsa_otbn_check_prime()` 调用 OTBN CHECK_PRIME。
   - 重复直到找到素数或达到 `max_tries = 20000`。
5. **找素数 Q**：与 P 相同流程。
6. **推导 N/D**：`rsa_otbn_key_from_pq()` 调用 OTBN KEY_FROM_PQ。
7. **CRT 参数**：`mbedtls_rsa_deduce_crt()`（未禁用 CRT 时）。
8. **校验**：`mbedtls_rsa_check_privkey()`。
9. 校验通过后把结果 swap 到 `ctx`。
10. 释放 session，清零栈缓冲区。

### 5.2 候选值生成

```c
static int rsa_otbn_gen_candidate(int (*f_rng)(void *, unsigned char *, size_t),
                                  void *p_rng,
                                  uint8_t *candidate_le,
                                  size_t prime_bytes)
{
    int ret = f_rng(p_rng, candidate_le, prime_bytes);
    if (ret != 0) {
        return ret;
    }

    /* 强制奇数，并设置最高两位，保证 p*q 达到目标位宽 */
    candidate_le[0] |= 0x01U;
    candidate_le[prime_bytes - 1] |= 0xC0U;

    return 0;
}
```

### 5.3 CPU 小素数试除

- 小素数表：`static const uint16_t rsa_keygen_small_primes[]`，共 1228 个素数，覆盖到 9973。
- 试除时按 MSB → LSB 解析 candidate，对每个小素数取模，若余数为 0 则丢弃。
- ROM 占用约 2.5 KB。

### 5.4 OTBN CHECK_PRIME

- 将 candidate 写入 DMEM offset `0x000`。
- 将模式写入 DMEM offset `0x720`。
- 执行 OTBN 命令 `OTBN_CMD_EXECUTE`。
- 从 DMEM offset `0x740` 读取 status；`0xFFFFFFFF` 表示素性通过。

### 5.5 OTBN KEY_FROM_PQ

- 将 p 写入 DMEM offset `0x000`。
- 将 q 写入 DMEM offset `0x200`。
- 将模式写入 DMEM offset `0x720`。
- 执行 OTBN。
- 从 DMEM offset `0x000` 读取 n，从 `0x200` 读取 d。

---

## 6. RNG 来源

### 6.1 keygen 用 RNG

必须使用真随机源，否则 candidate 分布有偏，可能找不到素数。

```c
static int rsa_keygen_get_rng(void *ctx, unsigned char *buf, size_t len)
{
    (void)ctx;
    if (trng_init() != 0) return -ENODEV;
    rc = entropy_get_entropy_isr(trng_dev, buf, len, ENTROPY_BUSYWAIT);
    return (rc < 0) ? rc : 0;
}
```

### 6.2 padding 用 RNG

PKCS#1 v1.5 要求非零随机字节，因此仍保留对随机字节做 `buf[i] |= 0x01` 的处理。

### 6.3 经验教训

- `samples/mbedtls_test` 原 `test_rng` 是确定性 LCG，且每个字节强制为奇数，**不能用于素数生成**。
- 已修复：keygen 单独使用 TRNG，padding/sign/verify 仍用 `test_rng`。

---

## 7. Hook 与回退机制

- `MBEDTLS_RSA_OTBN_HOOK` 在 `rsa.c` 中调用 `mbedtls_rsa_gen_key_otbn()`。
- 若 OTBN hook 返回非 0，`rsa.c` 会清零 `ret` 并继续执行 mbedtls 原有软件素数生成流程。
- 因此：
  - 不支持的配置自动回退软件；
  - OTBN 固件/运行异常不会导致 keygen 整体失败。

---

## 8. 性能基线（上板实测）

测试板：`lsqsh_evb@1os/lsqsh/cpu1`  
测试样本：`samples/mbedtls_throughput`  
算法：RSA-2048 keygen，公钥指数 F4

| 指标 | 10 次平均 | 备注 |
|------|----------|------|
| keygen 总耗时 | **~1286 ms** | 单次数值 535 ms ~ 2687 ms，取决于找到素数所需候选数 |
| SW RNG 调用 | ~903 次/次 | 进入 OTBN 前的 candidate 随机源调用 |
| OTBN RND 调用 | **~3901 次/次** | 加试除前单 loop 可达 62k+ |
| OTBN URND 调用 | **~1579 次/次** | 加试除前单 loop 可达 26k+ |

结论：加入 CPU 小素数试除后，RSA-2048 keygen 从约 15 s 级别降到约 1.3 s，**提升约 8~10 倍**，OTBN RND/URND 中断同步大幅下降。

> `samples/mbedtls_test` 中已增加 RSA-3072 / RSA-4096 的 keygen + sign/verify 功能验证，但尚未采集这两个位宽的耗时/RND 基线。

---

## 9. 已知问题与修复

| 问题 | 原因 | 修复 |
|------|------|------|
| `mbedtls_test` 报 `OTBN RSA keygen: failed to find prime P` | `test_rng` 是确定性 LCG，不适合素数生成 | 为 keygen 单独增加基于 TRNG 的 `rsa_keygen_get_rng()`，并在 overlay/prj.conf 中启用 `trng1` 和 entropy generator |

---

## 10. 后续可优化方向

按收益/改动量排序：

### 10.1 CPU 直接生成 p/q，OTBN 只跑 KEY_FROM_PQ（收益最高）

- CPU 侧完成 candidate 生成、试除、Miller-Rabin；
- OTBN 只做 `KEY_FROM_PQ`（mul + modinv_f4，完全不读 RND）；
- 预期 RSA-2048 keygen 可降到几百 ms；
- 风险：CPU 侧需满足 FIPS 186-5 A.1.3 全部约束，且不再由 OTBN 独立完成素数生成。

### 10.2 CPU 提供 Miller-Rabin base 给 OTBN

- 新增 `CHECK_PRIME_WITH_BASES` 模式；
- CPU 通过 DMEM 传入随机 base，OTBN 只负责模幂和判断；
- OTBN RND/URND 中断可降到 0；
- 收益中等，因为 OTBN 仍要对每个 survivor 做 4~5 轮模幂。

### 10.3 在 OTBN 内做 trial division

- 当前 candidate 生成和试除已在 CPU 完成，此改动收益小，反而增加 IMEM/DMEM 占用；
- **不建议**。

### 10.4 优化 OTBN 模幂/Montgomery 乘法

- 例如 sliding window、CIOS 变体等；
- 改动大、收益有限，优先级最低。

---

## 11. 附录：相关文件路径

```raw
modules/hal/linkedsemi/hal_driver/src/otbn/text_array/rsa_text.c
modules/hal/linkedsemi/hal_driver/inc/ls_otbn_rsa.h
modules/crypto/mbedtls/library/rsa_alt.c
modules/crypto/mbedtls/library/rsa.c            # hook 调用与回退逻辑
linkedsemi_zephyr_project/samples/mbedtls_throughput/src/main.c
linkedsemi_zephyr_project/samples/mbedtls_test/src/main.c
linkedsemi_zephyr_project/samples/mbedtls_test/boards/lsqsh_evb_cpu1.overlay
linkedsemi_zephyr_project/samples/mbedtls_test/prj.conf
.claude/plans/bubbly-seeking-sutton.md
```

---

## 12. 构建命令

```bash
# 性能测试样本
west build -p always -b lsqsh_evb@1os/lsqsh/cpu1 linkedsemi_zephyr_project/samples/mbedtls_throughput

# 功能测试样本
west build -p always -b lsqsh_evb@1os/lsqsh/cpu1 linkedsemi_zephyr_project/samples/mbedtls_test
```

> 当前 board 未配置 west flash runner，需使用现有烧录工具/脚本上板验证。

---

## 13. OTBN 固件侧补充（本仓库 `ls_otbn_rsa_keygen/`）

### 13.1 源码结构

| 文件 | 作用 |
|------|------|
| `src/main.s` | 入口模式分发、DMEM 变量定义（`mode`、`status`） |
| `src/rsa_keygen.s` | 核心流程：`GEN` / `COFACTOR` / `CHECK_PRIME` / `KEY_FROM_PQ`、素数候选生成、`derive_d`、`modinv_f4` |
| `src/primality.s` | Miller-Rabin 实现（`miller_rabin`、`miller_rabin_round`、`test_witness`） |
| `src/montmul.s` | Montgomery 乘法 |
| `src/mul.s` / `div.s` / `gcd.s` / `lcm.s` | 大数乘、除、GCD、LCM |
| `rsa_keygen_hal.h` | 固件与软件的接口契约：mode 值、DMEM 偏移等常量 |

> 当前 mbedtls hook **仅使用 `CHECK_PRIME` 和 `KEY_FROM_PQ`**；`GEN` 模式（OTBN 自生成 p/q）因 RND 中断过高已不再使用。
> `rsa_keygen_hal.h` 中的常量已迁移到 `modules/hal/linkedsemi/hal_driver/inc/ls_otbn_rsa.h`，作为软件侧调用依据。

### 13.2 Miller-Rabin 轮数

- RSA-2048（1024-bit prime，plen=4）：**5 轮**
- RSA-3072（1536-bit prime，plen=6）：**4 轮**
- RSA-4096（2048-bit prime，plen=8）：**4 轮**

轮数已按 FIPS 186-5 Table B.1 的最小要求配置，不建议再减。

### 13.3 RND/URND 使用点

- `generate_prime_candidate`：每个 256-bit limb 读一次 `RND` 和一次 `URND`。
- `miller_rabin_round`：每轮随机 base 生成同样按 limb 读 `RND` 和 `URND`。
- `KEY_FROM_PQ`、`derive_d`、`modinv_f4`：**不读随机数**。

因此当前架构里 OTBN 的随机数中断主要剩余来源是 `CHECK_PRIME` 的 Miller-Rabin base 采样。

如果要将 OTBN RND/URND 中断降到 0，可行路径与第 10 章优化方向一致：
- **CPU 直接完成 p/q 筛选**，OTBN 只跑 `KEY_FROM_PQ`；
- 或新增 **`CHECK_PRIME_WITH_BASES` 模式**，由 CPU 通过 DMEM 提供 Miller-Rabin base。

### 13.4 固件数组生成流程

`modules/hal/linkedsemi/hal_driver/src/otbn/text_array/rsa_text.c` 中的 IMEM 数组来源于本仓库：

```bash
# 反汇编 OTBN embed 对象
python3 hw/ip/otbn/util/otbn_objdump.py \
  ls_otbn_rsa_keygen/rsa_keygen.rv32embed.o \
  -g -e -t -G -s -S -x -D -r > ls_otbn_rsa_keygen/rsa_keygen.asm

# 从 .asm 提取 .rodata.otbn.text 生成 C 数组头
python3 ls_otbn_rsa_keygen/gen_firmware_h.py \
  ls_otbn_rsa_keygen/rsa_keygen.asm \
  ls_otbn_rsa_keygen/rsa_keygen_firmware.h
```

当前 `modules/hal/linkedsemi/hal_driver/src/otbn/text_array/rsa_text.c` 中的数组即由上述流程生成的 `rsa_keygen_firmware.h` 复制而来；后续若固件更新，需要重新同步到该文件。

当前固件大小：IMEM **4156 bytes**，DMEM 初始化数据为 0 bytes（运行时由 HAL 清零并写入输入）。

### 13.5 ISS 仿真验证

本仓库 `test_rsa_keygen.py` 使用 OTBN Python ISS 对关键路径做了与 Python `sympy` 的黄金参考对比：

| 用例 | 结果 | 对应软件阶段 |
|------|------|-------------|
| `cofactor_rsa2048_use_p/q` | PASS | — |
| `cofactor_rsa3072_use_p` | PASS | — |
| `check_prime_rsa2048_pass/fail` | PASS | `rsa_otbn_check_prime()` |
| `key_from_pq_rsa2048` | PASS | `rsa_otbn_key_from_pq()` |

验证命令：

```bash
python3 ls_otbn_rsa_keygen/test_rsa_keygen.py
```

> 注意：ISS 仿真 2048-bit Montgomery 模幂很慢（完整套件约 20 分钟），仅用于离线功能验证，不用于性能基准。
