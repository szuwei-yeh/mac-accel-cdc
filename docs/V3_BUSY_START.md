# V3 Step 2B：Busy-start admission control

## 修改 RTL 前選定的契約

以 bus-domain `job_in_flight` 記錄單一 software job 的生命週期：

- reset 清為 0；僅在 CTRL commit 同時滿足 low WSTRB byte enabled、WDATA[0]=1、
  合法 LENGTH、原 `job_in_flight=0` 時，回 OKAY、設為 1 並產生 start pulse。
- busy 時 start 回 SLVERR；非法 LENGTH 維持 SLVERR。兩種拒絕均不產生新的
  bus/MAC start 或 toggle，不重啟 DMA、不清除既有 sticky done/error/result/latency。
- zero strobe、low byte disabled、enabled low byte 但 bit 0=0 的 CTRL writes
  仍為 OKAY/no-op；不改 CSR map 或新增 status bits。
- 只在 bus edge 消耗 `done_pulse_bus`、同時捕捉 result/latency 時清為 0；
  不依 DMA done、FIFO empty 或同步 busy level 提早釋放。
- completion-consumption edge 的 pending CTRL start 使用 edge 前的 state：
  原 job 仍為 in flight，因此回 SLVERR。NBA capture 完成後，下一個 bus edge
  才能接受新 start；AW/W 提早到達不代表 admission 已發生。
- CTRL BUSY bit 改為 `job_in_flight | dma_busy | busy_bus`，補足 pending-start
  與 completion synchronization interval；既有同步 busy 鏈仍保留。
- BVALID/BRESP 在 BREADY=0 期間穩定，response 對應各自 write commit。

拒絕不會阻止合法 job 的自然進展：同 edge 的 done/result/error 更新仍可發生。
保留狀態的意思是拒絕本身不造成清除或替換，而非凍結正在執行的 job。

## 原始生命週期與風險

Step 2A top 第 172–177 行只驗證 LENGTH，未檢查 job 狀態；
第 347 行 `busy_any_bus = busy_bus | dma_busy` 僅為即時/同步 level 的 OR。
CTRL commit 產生 pulse 後，DMA 在下一個 bus edge 才鎖存 command；
同 edge start toggle 翻轉，再經 MAC two-flop chain 與 edge detection。
MAC busy 另經 bus two-flop chain 返回，所以不是原子的 admission state。

DMA 的 A phase 寫 `buf_a`，B phase 經 FIFO 的 full/backpressure 交付 operand pairs。
DMA 完成代表 B stream 已交付，不代表 FIFO/MAC pipeline 已完成。
MAC 處理 last 後產生 done；下一個 MAC edge 翻 done toggle，再經 bus two-flop
chain 與 delayed-bit XOR。BUSY level 可能先下降，完成事件與 bundled result/latency
仍未消耗。故僅加 `!busy_any_bus` guard 仍不能保證安全 release。

原始 DMA 只在 IDLE 接受 start；MAC 也只在 IDLE 處理 start，但 top 每個合法
CTRL start 都會回 OKAY、發 pulse/toggle 並清 sticky flags。忙碌時可能丟掉 job；
DMA 已 idle、MAC 尚未完成時可能重新發 memory traffic，且 toggle 也可能
在慢 MAC 捕捉前再次翻轉。以下會以定向測試區分實際觀察與結構推論。

## Snapshot

延續 worktree `/private/tmp/vector-mac-v3-wstrb`、分支
`fix/v3-axil-wstrb-20261011`，HEAD 維持
`762b8fae899d5b1f64183d91a30b3d58c34f2188`。
六個既有變更檔與 Step 2A inventory 的 hashes 一致；完整目錄副本，包括
untracked/ignored files，在 `/private/tmp/vector-mac-step2b-if20dkdc/step2a-snapshot`。
`step2a-inventory.json` 保存 hashes、status 與變更清單。

## 驗證與審查結果

相同 [15-case testbench](../sim/tb_mac_accel_dma_rob_busy.v) 對保存的 Step 2A top
執行：**3 PASS / 12 FAIL**，exit code 1；修正後 **15 PASS / 0 FAIL**，exit code 0。
前後使用同一份 testbench，SHA-256：
`f682939920f9cb14eca2fb650f3d8a7c2445a0492e5c285d2d79ee5ee662697a`。

| Case | 定向覆蓋 | 修正前 | 修正後 |
|---|---|---|---|
| 1 | Reset 後 start、commit 後 pending BUSY | FAIL：job 正常完成，但 pending BUSY=0 | PASS |
| 2 | 第一筆 commit 後第二個 edge 的最早 queued start | FAIL：額外 start/toggle | PASS |
| 3 | A DMA active | FAIL：額外 start/toggle | PASS |
| 4 | B DMA active | FAIL：額外 start/toggle | PASS |
| 5 | DMA 已完成、MAC 仍 active | FAIL：重啟 DMA、額外 AR/FIFO work | PASS |
| 6 | DMA idle、MAC active 且 FIFO 非空的 drain interval | FAIL：額外 AR/FIFO work | PASS |
| 7 | MAC start/busy 尚未到達 | FAIL：兩次 toggle 抵消，MAC start/done=0 | PASS |
| 8 | Done toggle 已改變、bus completion 尚未消耗 | FAIL：BUSY=0、額外 start/AR | PASS |
| 9 | CTRL commit 與 done consumption 同 edge | FAIL：提前接受 start | PASS |
| 10 | Capture 後第一個 bus edge 接受新 job | PASS | PASS |
| 11 | 三次 rejected starts、每次 B stall 12 cycles、真實 RRESP error fixture | FAIL：額外 start，sticky DMA error 被清除 | PASS |
| 12 | Busy rejection 後完成舊 job，再接受新 job | FAIL：額外 start events | PASS |
| 13 | Active job 期間把 LENGTH 改為 0 再 start | PASS：Step 2A guard 有效 | PASS |
| 14 | Busy 期間 zero/mixed-disabled/bit0-clear no-op 與 low-byte start | FAIL：low-byte busy start 被接受 | PASS |
| 15 | 三筆合法 jobs、無 inter-job reset | PASS | PASS |

修正前 case 5/6 的 length-16 job 出現 4 個 AR、64 個 R beats、32 個 FIFO writes；
正確應為 2/32/16。Case 7 的 bus starts/toggles=2，但 MAC start/done=0。
這些是實際失敗紀錄，並非僅依 RTL 推測。

scoreboard 的 accepted/rejected/no-op 數量以預定介面契約獨立記錄；
bus pulse、toggle transition、MAC pulse、MAC/bus done、AR/R 與 FIFO read/write
另行觀察並比對。每筆接受的 job 有獨立 signed operand/result plan，核對
AR address/geometry、FIFO payload/order/last、精確 beat/event conservation、
result/latency capture 與 sticky flags。拒絕不凍結舊 job，允許其正常 memory
traffic 與同 edge 的合法 completion；任何超出 accepted plans 的 AR/operand
或完成均視為失敗。RRESP error fixture 只用於 testbench，不改 public RTL。

AW/W/AR/R/B 均有 80-cycle timeout；job/phase 等待有 10000 bus-cycle 上限，
另有 process timeout 與全域 simulation timeout。修正後全部無 timeout。
新 case 11 的標題避免 diagnostic keyword `error`，既有 CI failure detector
維持不變。等週期壓力測試曾揭露觀測計數器跨 case 殘留：觀測 reset 改成
跟隨真實 asynchronous MAC reset；未更動預期值或額外修改 RTL。
最終同版 testbench 已對 snapshot 與修正版重跑；早期探索 logs 在 external
`initial-harness/`，不是最終 regression 證據。

## Admission 實作與完成邊界

[V3 top](../rtl/mac_accel_dma_rob_top.v) 第 131 行新增一個 bus-domain register，
第 148 行 reset，第 179–181 行在合法 idle CTRL commit 設為 1。
第 155 行以原有 `done_pulse_bus` 清為 0，同 edge 第 396–398 行捕捉
result/latency；admission condition 使用原 state，故此 edge 的 start 回 SLVERR。
第 353 行擴充 BUSY bit 的來源。原 synchronizer topology、DMA/ROB、FIFO、MAC
pipeline、AXI handshake FSM、register map 與其他 reset 邏輯未改動。

預設邊界紀錄：done-sync-window commit tick 308（done=0），completion-edge
commit tick 309（done=1）皆拒絕；capture 後 tick 310（done=0）接受。
修正前前兩者的 `dma_busy=0`、`busy_bus=0`、`busy_any_bus=0`；修正後
前兩者 BUSY=1，下一個安全 edge BUSY=0。最早 queued 第二筆 commit tick 25，
第一筆 tick 23，MAC start 尚未到達；不能宣稱第二筆能在第一筆 DMA busy
assertion 前 commit，因現有 AXI-Lite response/commit pipeline 不允許。

Boundary stimulus 以 read-only internal observation 排程 AXI input，沒有 force、
deposit 或修改 DUT state。Case 8 在 done-toggle source event 後 1 ps 送出，
case 9 在 first/second sync stage 差異的 negedge 準備 AW/W，case 10 在
done-pulse negedge 準備；在實際 commit edge 分別檢查 phase predicate。
FIFO drain 是 active drain interval，未宣稱 CTRL 與某個 MAC read edge 同時。

六種 base clock/initial-phase configurations，每種 15 cases 均 PASS，合計
**90/90**，含同 edge reject 與 next-edge accept 的精確 predicate：

| Base MAC period (ns) | Clock-generator initial phase (ns) | Cases |
|---:|---:|---:|
| 7.500 | 0.000 | 15/15 |
| 17.000 | 1.250 | 15/15 |
| 10.000 | 0.000 | 15/15 |
| 10.000 | 2.500 | 15/15 |
| 7.502 | 1.250 | 15/15 |
| 3.500 | 0.375 | 15/15 |

Bus period 固定 10 ns；cases 2/7 使用 200 ns MAC，cases 4/5/6/11/14 使用
100 ns MAC，以穩定保留指定窗口；其餘使用表列 base period。
clock period 切換發生於 case reset 期間；phase 欄是 generator 初始 offset，
不是宣稱所有 cases 都維持同一 phase offset。這是數位 scheduling stress，
不代表 metastability/MTBF/RDC signoff。

## 本次回歸與 Step 2B-only diff

| 本機 Icarus 12.0／Python 3.13 檢查 | 結果 |
|---|---|
| Busy-start directed | 15/15 PASS；六配置 90/90 |
| 原始 Step 2A LENGTH | 13/13 PASS |
| 原始 Step 1 WSTRB | 26/26 PASS |
| V3 full-top | 7/7 PASS |
| ROB OOO/interleaved | 12/12 PASS |
| ROB DMA wrapper | 11/11 PASS |
| CPU-fed MAC/CDC | 8/8 PASS |
| In-order ROB | 4/4 PASS |
| Power-workload RTL | 4/4 PASS；未重估 power |
| V3 CDC matrix | 6/6 scenarios、36/36 jobs PASS |
| Python syntax／unit tests | 39 files／13 tests PASS |
| Whitespace，包括新增檔案 | PASS |
| Documentation/pinned integrity | FAIL：僅 V3 top historical source hash mismatch |

相對 Step 2A snapshot，四個檔案有 Step 2B changes：

- `rtl/mac_accel_dma_rob_top.v`：+10/−4 lines，一個 admission bit、set/clear/guard、BUSY。
- `scripts/run_ci_rtl.py`：+4/−0 lines，加入要求精確 15 PASS / 0 FAIL 的 entry。
- `sim/tb_mac_accel_dma_rob_busy.v`：新增 545 lines。
- 本審查紀錄。

既有 WSTRB/LENGTH tests 與兩份報告保持原 hash；除了 top/runner，其餘 snapshot
的 published files 全部 byte-identical。Step 2B-only `step2b.diff`、inventory、
前後 logs、clock-stress JSON、commands 與 hashes 在
`/private/tmp/vector-mac-step2b-if20dkdc/`；CDC reports 在
`/tmp/vectormac-cdc-supp-khr0w8fn/reports/`。未 commit、merge、push。

主要檢查可用既有命令重現：

```bash
python3 scripts/run_ci_rtl.py
python3 sim/run_cdc_regression.py --simulator iverilog
python3 scripts/check_portfolio.py --python-syntax
python3 -m unittest discover -s tests -v
git diff --check
python3 scripts/check_portfolio.py
```

最後一項預期 exit code 1；沒有修改或 bypass pinned check。
Busy test binary 可追加 `+MAC_PERIOD=7.502 +MAC_PHASE=1.250` 等 plusargs
重現 clock stress；編譯 source list 與選定 top 在 CI runner 中。

## Provenance、剩餘風險與下一步

V1/V2、`physical/`、`formal/`、pins、manifests、templates、accepted reports、
acceptance checks、README presentation 全部未修改。
歷史 top hash 是 `f811c3fad110563e0a1b930b4955357a349b129852291e8f3acd05c5eb3d32fd`；
Step 2A 是 `33e3f52b148663abcac3d8c9f54b157e9ad86acf39bcb267922f5e26ac393127`；
本次是 `0f02ea10217a1138eab7f24c468acde9bd352de156e23db7df4bf383ce92e9a1`。
這是預期 historical-snapshot difference；歷史 GDS/PPA/STA/GLS/formal 證據
仍屬於各自原始 source/model，未重新標示為修正版成果。

VCS、SymbiYosys、DC、PrimeTime、OpenROAD/OpenSTA 不在本機 PATH；未執行
synthesis、formal、mapped GLS、physical timing、P&R 或 archived physical acceptance。

仍假設正常運作的 clocks、協調的 reset 與合法 memory responses。job 若永久
無進展，admission bit 不會自行釋放；本步沒有 watchdog/abort 或單域 reset
recovery。CSR configuration writes 仍可在 active job 期間更新；active DMA 使用
已鎖存 configuration，下個 start 使用 CSR commit 時的值。一般參數化、address
alias、未映射 writes 與 4 KB burst boundary 等原有範圍未擴充。

最終 RTL robustness audit 建議先審查獨立 AW/W back-to-back handshakes、
response queue 在 completion 附近的 commit semantics、configuration snapshot、
reset/recovery 與 AXI address/burst contract。保留本次逐步 diff 與歷史 evidence
界線，再另行規劃新增 formal 與修正版 synthesis/physical validation。
