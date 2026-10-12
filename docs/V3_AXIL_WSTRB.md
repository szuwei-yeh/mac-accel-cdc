# V3 AXI4-Lite WSTRB 修正審查紀錄

2026-10-11；僅修正 V3 CSR write-strobe handling。

## 基準與範圍

遠端 `refs/heads/main` 經 `git ls-remote` 與 `git fetch origin main` 確認為
`762b8fae899d5b1f64183d91a30b3d58c34f2188`。
獨立 worktree：`/private/tmp/vector-mac-v3-wstrb`；分支：
`fix/v3-axil-wstrb-20261011`。未 commit、merge、push；其他 worktree 未修改。

修改檔案：

- [V3 top RTL](../rtl/mac_accel_dma_rob_top.v)：CSR byte enables。
- [定向 testbench](../sim/tb_mac_accel_dma_rob_axil_wstrb.v)：26 項自動比對案例。
- [既有 CI runner](../scripts/run_ci_rtl.py)：加入定向測試，要求精確 26 PASS / 0 FAIL。
- 本審查紀錄。

## 根因與修正

基準 RTL 第 160–161 行鎖存 WDATA/WSTRB，但第 170 行僅以
`w_data_r[0]` 觸發 start，第 172–174 行直接覆寫 A/B/LENGTH。
因此 WSTRB 雖被保存，卻未參與 CSR 提交，亦被列入 unused tie-off。

修正版第 171 行要求 `w_strb_r[0] && w_data_r[0]` 才產生 start；
第 173–181 行依 byte enable 更新 A/B，各未選取 byte 保留；
第 183–185 行只以 strobe 0/1 更新 LENGTH 的兩個 byte。
零 strobe 仍正常回應 OKAY，但沒有 CSR 或 start 副作用。
刪除 WSTRB 的 unused tie-off。

AW/W 仍分別鎖存，兩者齊備且無 pending B response 時才提交；
BVALID/BRESP 在 BREADY 停滯期間維持穩定，B handshake 後清除。
既有 READY、response、reset 與 CDC/DMA/ROB/FIFO/MAC 邏輯未更動。

## 先失敗、後通過

同一份 testbench、相同預期值，在改 RTL 前執行：**7 PASS / 19 FAIL**，
模擬以 `$fatal` 回傳非零 exit code。失敗分布：

| 案例 | 失敗數 | 基準實際行為 |
|---|---:|---|
| 零 strobe：A/B/LENGTH/CTRL | 4 | 覆寫暫存器或觸發 start |
| A/B：低 byte、高 byte、mixed lanes | 6 | 覆寫整個 word，未保留 disabled lanes |
| LENGTH：lane 0、lane 1、上兩 lanes、mixed | 4 | 直接覆寫低 16 bits |
| CTRL：高 byte、mixed 且未啟用低 byte | 2 | WDATA[0]=1 仍觸發 start |
| AW 先於 W、W 先於 AW 的 partial write | 2 | 提交資料正確配對，但忽略 strobe |
| B 停滯的 partial write | 1 | response 穩定，但 CSR 忽略 strobe |

原始 7 個通過案例：A/B/LENGTH 全字寫入、CTRL 低 byte/full word 各一次 start、
CTRL bit 0 清零，以及 stalled B 後方排隊的全字寫入。
修正後：**26 PASS / 0 FAIL**，exit code 0。

每個 AW/W/AR/R/B 等待均有 80-cycle 限制，另有全域 timeout。
測試在 negedge 驅動、posedge 計算 handshake，使用明確常數預期值與 CSR readback；
核對 AW/W/B 數量、start pulse 次數/寬度與 toggle。
AW/W 接受後改變 live payload，檢查提交使用已鎖存資料與 strobe。
另檢查 B 停滯 12 cycles、CTRL start 不重複，以及第二筆寫入不得在前一筆
B response 消耗前提交。
非操作範圍的 LENGTH 位元樣式只測儲存，不發出 start。

## 本次實際執行結果

使用本機 Icarus Verilog 12.0 與 Python 3.13；所有模擬均為 RTL。

| 檢查 | 結果 |
|---|---|
| 新增 WSTRB regression，depth 8 | 26/26 PASS |
| V3 full-top，depth 8 | 7/7 PASS |
| OOO/interleaved ROB，depth 8 | 12/12 PASS |
| ROB DMA wrapper，depth 8 | 11/11 PASS |
| CPU-fed MAC/CDC | 8/8 PASS |
| V3 dual-clock CDC matrix | 6/6 scenarios；36/36 jobs PASS |
| In-order ROB，depth 8 | 4/4 PASS |
| Deterministic power-workload RTL，depth 8 | 4/4 PASS；未重估 power |
| Python syntax | 39/39 files PASS |
| Python acceptance/scoreboard unit tests | 13/13 PASS |
| `git diff --check` | PASS |
| `check_portfolio.py`，修正前 | PASS，181 local links 與 pins 一致 |
| `check_portfolio.py`，修正後 | FAIL：僅 V3 top pinned source hash mismatch |

可從 repository root 重現主要檢查：

```bash
python3 scripts/run_ci_rtl.py
python3 sim/run_cdc_regression.py --simulator iverilog
python3 scripts/check_portfolio.py --python-syntax
python3 -m unittest discover -s tests -v
git diff --check
python3 scripts/check_portfolio.py
```

最後一項預期仍會以 exit code 1 報告 V3 top 的歷史來源差異；沒有 bypass。

完整本機 logs、基準 RTL snapshot 與 review diff 保存於
`/private/tmp/vector-mac-v3-wstrb-evidence/`，包含 `before.log`、`after.log`、
`rtl.log`、`cdc.log`、`inorder.log`、`power.log`、`unit.log`、
`python-syntax.log`、`baseline-integrity.log`、`integrity.log` 與 `review.diff`。
CDC runner 的 source hashes、逐 job 紀錄與事件 CSV 保存在
`/tmp/vectormac-cdc-supp-um2r2wxe/reports/`。
這些是本機暫存紀錄，未發布為 hosted CI 或歷史 acceptance evidence。

## 歷史證據與後續事項

`physical/`、`formal/`、所有既有 testbench 與其預期結果均未修改。
`physical/pins.json`、templates、manifests、acceptance checks 與 artifacts 原封不動。
歷史 Nangate45 snapshot 的 top SHA-256：
`f811c3fad110563e0a1b930b4955357a349b129852291e8f3acd05c5eb3d32fd`。
修正後 top SHA-256：
`7b94273951198ed4653b5642f0771e1a7336cc1ca4fc84f4d1cec53b849c0c00`。
這是預期的 historical-snapshot difference。既有 GDS/PPA/GLS/STA 證據仍屬於
原始 snapshot，不能宣稱已涵蓋本次修正。

VCS、SymbiYosys、DC、PrimeTime、OpenROAD/OpenSTA 不在本機 PATH；
未執行 synthesis、formal、mapped GLS、physical timing 或 P&R。
需完整 archived bundle 的 `physical/test_acceptance.py` 未執行。

尚未處理非法 LENGTH、busy 時 start、一般參數化，以及地址高位/非對齊 alias
與未映射寫入仍回 OKAY 的既有行為；本測試不宣稱完整 AXI-Lite protocol signoff。
建議下一步另建分支，針對 LENGTH=0 與 LENGTH>MAX_LEN 定義行為、先寫失敗測試，
再做最小修正。
