# V3 Step 3B：DMA error ownership 修正

完成 F01；只改 wrapper 的 error 累積條件，沒有新增 state，也沒有改 read engine、ROB、FIFO、MAC、CDC、CSR map 或 admission。

## 根因與逐拍時序

`rtl/mac_dma_rob.v` 原第180–181行無條件累積 `engine_err`；第189–205行在start清除wrapper error並進CMD_A。`rtl/axi_read_engine_rob.v` 第206–215行到command acceptance才清 engine sticky error，第244–251行在R handshake設error，第280–285行在ordered output最後beat後才通知完成。Top第405–410行會重新累積wrapper sticky bit。

| Bus edge | 原實作 | 修正後 |
|---|---|---|
| E0：CTRL commit | 接受下一工作、產生start | 相同 |
| E1：wrapper消耗start | 清dma_err與top error；engine仍留前一B錯誤，進CMD_A | 相同 |
| E2：A command acceptance | wrapper讀舊engine_err=1；同edge engine NBA清為0；wrapper又設dma_err=1 | CMD_A不累積error；進RUN_A時engine已清為0 |
| E3 | top重新設sticky error | 乾淨工作保持0 |

Engine sticky error自當前command第一個non-OKAY R handshake後有效，保持至下一command acceptance；在idle不代表新工作已出錯。A error會在RUN_A累積並跨B command保持；B error會在RUN_B累積，完成後仍sticky，直到下一start清除。

## 最小修正與最後B beat

`rtl/mac_dma_rob.v` 現第180–183行改成：

```verilog
// RUN phases own the accepted command's sticky error. CMD_A may
// still see the previous job's error until the engine accepts it.
if (engine_err && (state == S_RUN_A || state == S_RUN_B))
    dma_err <= 1'b1;
```

不能在接收最後R beat的同edge退出RUN_B：engine先NBA保存資料與e_complete/error；之後ordered retirement才消耗最後output beat，進engine S_DONE，再發engine_done，wrapper下一edge才轉S_DONE。因此最後R錯誤在wrapper離開RUN_B之前可被取樣。

最小length1實際trace：tick45最後B R SLVERR握手，engine_err_before=0；tick46 wrapper仍state4且engine_err=1/dma_err=1（有assert）；tick48 wrapper才消耗engine_done。第一job CTRL=0xC、sum=-6。第二job tick86清wrapper/top error；tick87 CMD_A接受後engine/dma/top均0；第二job CTRL=0x4、sum=-3。Length16 final-B-only與length17/65 partial-final錯誤亦通過，測試要求fault counters非零。

## 永久回歸與修正前後

新增 `sim/tb_mac_accel_dma_rob_error.v`，從Step3A最小reproducer的bounded AXI-Lite drivers出發。Response fault按獨立AR address／RID metadata／element plan選擇，不依DUT phase注入；一beat buffer維持合法RVALID與payload，stall期間有完整RID/DATA/RESP/LAST stability assertions。每scenario可startup reset，但2或3個jobs之間不reset。

相同testbench hash：`b220ff6d4a28231ad0c9da629e54d4c34fa514568e6dea0228637572cb373949`。原Step2B **3 PASS / 9 FAIL，exit1**；修正後 **12 PASS / 0 FAIL，exit0**。

| 情境 | 修正前 | 修正後 |
|---|---|---|
| 01 minimal final B fault -> clean len1 | FAIL | PASS |
| 02 clean -> clean len16/17 | PASS | PASS |
| 03 A fault -> clean len1/17 | PASS | PASS |
| 04 B faults -> clean len16 | FAIL | PASS |
| 05 A+B faults -> clean len17 | FAIL | PASS |
| 06 fault -> clean -> fault | FAIL | PASS |
| 07 fault -> fault -> clean | FAIL | PASS |
| 08 long partial burst with gaps/full FIFO | FAIL | PASS |
| 09 final B only len16 -> clean17 with gaps | FAIL | PASS |
| 10 busy rejection preserves active fault | FAIL | PASS |
| 11 accept next job first safe completion edge | FAIL | PASS |
| 12 final A only len17 -> clean16 with gaps | PASS | PASS |

12情境共26jobs，檢查signed result／CTRL done-error-busy、per-job及總AR/R/FIFO counts、一次start/toggle/MAC done/bus done、sticky與clear邊界、final A/B error、busy rejection及第一safe edge admission。Case08/10明確要求output_stalls>0；Case06–12要求response gap coverage。RREADY在engine RUN恆高，未虛稱測到不存在的RUN RREADY backpressure；source buffer另對behavioral memory施加stall及response gaps。Case11第一job completion用internal capture assertions，第二job作CSR讀回；最小Case01兩job各以公開CSR核對0xC/0x4。

## 驗證結果

- 新error12、WSTRB26、LENGTH13、busy15、V3top7、ROB OOO12、DMA ROB11、V1core8：CI共104/104。
- ROB in-order4、power workload功能4、V2top7、V2backpressure3、V2baseline5、DMA sweep8：補充31/31；功能案例總135/135（不含CDC jobs）。
- V3 CDC Icarus：6/6情境、36/36jobs。
- Python syntax39files、unit13/13、git diff --check：PASS。
- Verilator lint與Yosys proc/opt/check -assert：depth4/8 PASS，既有EOFNEWLINE/SYNCASYNCNET warnings；無generic latch或結構衝突。沒有mapped synthesis／timing/PPA宣稱。
- V1 F02/F03原probe重新執行：各exit1，仍是已知缺陷；本步未修V1/V2。
- Portfolio check exit1只列以下兩項預期historical source差異，未繞過檢查。

## Provenance與review artifacts

Git baseline `762b8fae899d5b1f64183d91a30b3d58c34f2188`；physical RTL revision `38d6353712ac7ba680868f0729cf9006e7bff93d`。外部原始182file快照／metadata與所有logs：`/var/folders/02/btvzh8n52_3fv883t3ym38mr0000gn/T/vector-mac-step3b-3umnbba3`，pre-fix snapshot包含既有untracked檔；另加同版新TB供重現。

| Source | Historical SHA-256 | Current SHA-256 |
|---|---|---|
| `rtl/mac_accel_dma_rob_top.v` | `f811c3fad110563e0a1b930b4955357a349b129852291e8f3acd05c5eb3d32fd` | `0f02ea10217a1138eab7f24c468acde9bd352de156e23db7df4bf383ce92e9a1` |
| `rtl/mac_dma_rob.v` | `38f18f58d541c70e5e2b97130e7dca0e1c74f5bffcd9cd74b24d3c2218ba0f32` | `9042b7fbb17ce7aeeb42763899aeb042597a48f7f88d47d87512f3f0f5aa7eb8` |

`physical/`、`formal/`及Step1/2A/2B其他既有檔案逐一hash未變；runner僅新增一個12case entry，原check_scoreboard／golden counts不變。Step3B-only diff：`/var/folders/02/btvzh8n52_3fv883t3ym38mr0000gn/T/vector-mac-step3b-3umnbba3/step3b-only.diff`；修改清單為wrapper、runner、新TB與本報告。未commit、merge或push，未改其他既有worktree。

歷史OSU block timing/power與Nangate45 full-top PPA只屬原source。此修正新增wrapper hash差異，不能沿用舊PPA當現行RTL量測；新revision的synthesis須另建manifest。

## 剩餘問題與下一步

F01已修正。V1 F02/F03、ROB OOO completion非空性F04、qualified parameter domain仍需各自独立處理；新overnight授權會在不同V1 worktree修復，並先凍結此V3候選，再建立獨立synthesis experiment。本報告是Stage A結果，不代表之後synthesis已完成。
