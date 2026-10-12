# V3 Step 2A：非法 LENGTH 契約與驗證

## 修改 RTL 前選定的介面契約

只有 `1 <= LENGTH <= MAX_LEN` 可啟動 job。判斷在 CTRL write 提交時，
使用當時的 16-bit LENGTH CSR；各 byte 的儲存仍遵循 Step 1 WSTRB。

- LENGTH write 本身仍回 `OKAY (00)`，包含非法值，方便軟體分次更新 bytes。
- CTRL 的 low byte enabled 且 WDATA[0]=1 才視為 start request。
- 合法 start 回 `OKAY (00)` 並執行原本的 start pulse。
- 非法 start 回 `SLVERR (10)`，不產生 bus/MAC start，不啟動 DMA、不發 memory
  requests、不寫 FIFO、不執行 MAC；不新增 CSR、不借用 DMA error bit。
- CTRL 的 zero strobe、未 enabled low byte 或 WDATA[0]=0 仍回 OKAY，無 start。
- 拒絕時保留既有 done/error/result/latency。若前一筆合法 job 已完成，sticky
  done=1 仍代表該筆 job；拒絕不得新增 done event 或將舊結果視為新完成。
- BVALID/BRESP 在 BREADY=0 期間保持穩定；response 只在新 write 提交時設定。

SLVERR 只需現有 commit 分支內的範圍判斷與 response 覆寫，無須 FSM 重構。
非法請求開始回錯誤是有意的 public interface 行為變更；合法軟體行為不變。
busy-start admission control 留給 Step 2B。

## Step 1 保留與檢查

沿用 `/private/tmp/vector-mac-v3-wstrb`，分支 `fix/v3-axil-wstrb-20261011`，
HEAD 保持 `762b8fae899d5b1f64183d91a30b3d58c34f2188`。
改動前，四個 Step 1 檔案均與先前交付的 SHA-256 inventory 一致。
包含 untracked/ignored files 與 `.git` worktree pointer 的完整工作目錄副本在
`/private/tmp/vector-mac-step2a-w5685wti/step1-snapshot`；
`step1-inventory.json` 記錄 published files hashes、Git status 與變更清單。

原 Step 1 沒有獨立 full-word AW-first case；full-word W-first 只在 queued
write case。以保存的 driver 額外執行兩個獨立 full-word cases：**2/2 PASS**，
無功能問題；原 26-case testbench 未修改。此補充紀錄在
`/private/tmp/vector-mac-step2a-w5685wti/fullword-before.log`。

## 修正前的 RTL 行為追蹤

Step 1 top 第 171 行只檢查 strobe 0 與 WDATA[0]；第 184–185 行可儲存所有
16-bit LENGTH，第 191–192 行一律回 OKAY。因此非法值也會產生 start，
清除既有 sticky done/error，並經 start toggle 啟動 MAC。

[DMA](../rtl/mac_dma_rob.v) 第 189–198 行不驗證 LENGTH，直接鎖存並進入 CMD_A。
[read engine](../rtl/axi_read_engine_rob.v) 第 206–220 行接受任何 command；
第 137–157 行以剩餘元素判斷 ARVALID，RUN 時 RREADY=1；第 179–182、274–280 行
只在最後一個有效 retirement 後完成。

| LENGTH／情境 | 修正前推論，由 RTL 結構確定 |
|---|---|
| 0 | AXI-Lite OKAY/start；DMA RUN_A、engine RUN、RREADY=1；ARVALID=0，無 ROB allocation/retirement，沒有 done；MAC 無 last 而維持 busy |
| 1、16、17、MAX_LEN | 正常 A/B phases、ROB issue/retire、FIFO 配對、MAC last/done |
| MAX_LEN+1、65535 | OKAY/start，engine 依 LENGTH 發 AR、接 R、配置/退役 ROB；wrapper 不限制 buffer 索引 |
| partial WSTRB LENGTH | 先合併 selected bytes；CTRL 使用合併後的值，原本沒有範圍檢查 |
| valid completion 後 invalid | start 清 sticky done/error；先前 result/latency CSR 暫留，之後行為依非法 LENGTH 而定 |
| invalid=0 後 valid、無 reset | DMA/MAC 還在 RUN；DMA 非 IDLE 不接受 start，top 仍回 OKAY 並切換 start event；不能靠下一個 start 正常恢復 |

DMA 第 79、157–158、211、229 行以縮窄索引讀寫 A-buffer。預設 MAX_LEN=256
時超長 A phase 在第 257 筆覆寫 entry 0，B phase 也會繞回，因此資料配對可能
錯誤；不是聲稱此預設配置發生 out-of-bounds array access。
超長完整 job 是否完成，仍取決於 memory responses、FIFO/MAC 進展與合法 AXI
環境；本次測試不宣稱完整跑完 65535 元素。非 power-of-two MAX_LEN 的額外
參數化風險不在本步處理。
非法 LENGTH 不會自行產生 DMA error；engine error 只檢查收到的 RRESP。

以上為 RTL 追蹤。修正前的實際模擬另確認：LENGTH=0 的觀察窗內 start +1、
AR/R/FIFO +0，DMA state=RUN_A、engine state=RUN、busy=1、done=0；
257/65535/partial 合併成 272 各有 start +1、8 個 accepted AR、115 個 R beats，
且仍在 RUN_A。這些有限時間觀察證實非法請求啟動資料路徑，並未將全部超長
job 的最後結果當成已測量證據。

## 驗證紀錄

相同 [13-case testbench](../sim/tb_mac_accel_dma_rob_length.v) 先在未修改的
Step 1 RTL 上執行，結果 **6 PASS / 7 FAIL**，exit code 1；修正後
**13 PASS / 0 FAIL**，exit code 0。前後 testbench SHA-256 相同：
`145eb7248e1e2f4cae6870fa2590140529e9b755abe384c327753214da536811`。
沒有調整 expected values 來配合 RTL。

修正前七個失敗案例：0、MAX_LEN+1、65535 各含下一筆合法 recovery；
invalid→valid 無 reset；valid→invalid→valid 的 sticky done；partial WSTRB
將 LENGTH 合併為 272，再修回 16；以及 stalled SLVERR 後方 queued LENGTH write。
四個合法長度 1/16/17/256，以及非法 LENGTH 下 CTRL zero strobe／low byte
disabled 的兩個 no-op cases 原本已通過。

每個 invalid case 檢查 BRESP、bus/MAC start、ARVALID 與 accepted AR、RREADY/R、
ROB state/occupancy/issue/retire/pointers、buffer indices、FIFO write/read、MAC
busy/pipeline、done events/toggles，以及 status/result/latency 保留。
每次拒絕後均在不 reset 的情況驗證下一筆合法 job；partial 修復直接發 start，
沒有先以 full-word write 蓋掉修復結果。100 bus-cycle 觀察窗之外，再檢查 recovery
的精確 start/done、burst/beat/retirement/FIFO 數量與 dot product。
合法 1/16 cases 分別獨立使用 full-word AW-first／W-first。
AW/W/AR/R/B 有 80-cycle timeout；合法完成有 5000-cycle 上限與 fresh done event
判斷；另有全域 timeout。修正前卡住的 recovery 被記錄為 bounded-wait failure，
未阻止其餘 cases；修正後無 timeout。

本機 Icarus 12.0／Python 3.13 實際結果：

| 檢查 | 結果 |
|---|---|
| 新 LENGTH tests，MAX_LEN=256、depth 8 | 13/13 PASS |
| Step 1 原始 WSTRB regression | 26/26 PASS |
| V3 full-top，depth 8 | 7/7 PASS |
| ROB OOO/interleaved，depth 8 | 12/12 PASS |
| ROB DMA wrapper，depth 8 | 11/11 PASS |
| CPU-fed MAC/CDC | 8/8 PASS |
| In-order ROB，depth 8 | 4/4 PASS |
| Deterministic power-workload RTL，depth 8 | 4/4 PASS；未估算 power |
| V3 CDC matrix | 6/6 scenarios、36/36 jobs PASS |
| Python syntax | 39 files PASS |
| Acceptance/scoreboard unit tests | 13/13 PASS |
| `git diff --check` 與新增檔案 whitespace | PASS |
| 完整 documentation/pinned hash check | FAIL：僅 V3 top historical hash mismatch |

主要重現命令：

```bash
python3 scripts/run_ci_rtl.py
python3 sim/run_cdc_regression.py --simulator iverilog
python3 scripts/check_portfolio.py --python-syntax
python3 -m unittest discover -s tests -v
git diff --check
python3 scripts/check_portfolio.py
```

新 test 已加入既有 CI runner，要求精確 13 PASS / 0 FAIL。
最後的 pinned check 預期仍為 exit code 1，沒有任何 bypass。
前後 logs、commands、hashes 與 Step 2A-only `step2a.diff` 在
`/private/tmp/vector-mac-step2a-w5685wti/`；CDC source snapshots、逐 job 統計與
事件 CSV 在 `/tmp/vectormac-cdc-supp-kzj3epuo/reports/`。

## Step 2A-only 修改

相對 Step 1 snapshot，僅以下四檔：

- [V3 top](../rtl/mac_accel_dma_rob_top.v)：+8/−2 lines。在第 174 行限制 LENGTH，
  非法時第 177 行回 SLVERR。把 default OKAY 移至 commit 開頭第 169 行，避免
  後續無條件賦值覆蓋 SLVERR；既有 BVALID/READY、reset 與所有 datapaths 不動。
- [CI runner](../scripts/run_ci_rtl.py)：+4/−0 lines，增加 13-case entry。
- [LENGTH testbench](../sim/tb_mac_accel_dma_rob_length.v)：新增 401 lines。
- 本契約、分析與審查紀錄。

Step 1 的 WSTRB testbench 與紀錄維持原 hash；其他 snapshot 中的 published
files 均 byte-identical。未 commit、merge、push，未修改其他 worktree。

## Provenance 與剩餘風險

`physical/`、`formal/`、V1/V2、既有 testbench/expected values、README presentation
均未修改。歷史 pins、templates、manifests、acceptance checks 原封不動。
歷史 top SHA-256 為
`f811c3fad110563e0a1b930b4955357a349b129852291e8f3acd05c5eb3d32fd`；
Step 1 為 `7b94273951198ed4653b5642f0771e1a7336cc1ca4fc84f4d1cec53b849c0c00`；
本次為 `33e3f52b148663abcac3d8c9f54b157e9ad86acf39bcb267922f5e26ac393127`。
這是預期 historical-snapshot difference。既有 GDS/PPA/STA/GLS/formal 記錄
仍屬於各自原始 source/model，不代表新 top 已經重新驗證。

VCS、SymbiYosys、DC、PrimeTime、OpenROAD/OpenSTA 未在本機 PATH；
未重跑 synthesis、formal、mapped GLS、physical timing 或 P&R。
需完整 archived bundle 的 physical acceptance suite 未執行。
本次是有限情境 RTL 驗證，並非完整 AXI-Lite、CDC/RDC 或 formal signoff。

本 guard 僅保護 V3 CSR start 入口；直接使用 DMA/read-engine blocks 仍需呼叫端
保證合法長度。busy 時合法 start 仍沿用原行為，CSR 可在執行中修改，address
alias／未映射 writes／一般參數化／4 KB burst contract 均未擴充。

Step 2B 建議先定義單一 job 的 admission 與 rejection response，並測試 active
DMA、DMA 完成但 MAC 仍 active、start 後 CDC busy 尚未返回及 completion 邊界。
需確保 busy 時不產生任何新 start toggle、不清既有 job 狀態，且下一筆 idle
start 可接受；不可僅依晚到的 MAC busy synchronizer 決定 admission。
