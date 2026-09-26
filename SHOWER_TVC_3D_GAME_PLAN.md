# FLEXIBLE HOSE SHOWER CONTROL GAME PLAN

## 0. Decision record

Side Lab X1 の本命を、剛体シャワーヘッドの姿勢制御から **内部流れをもつ柔軟ホースの流体構造連成**へ切り替える。

Human Visual Audit と実演例の確認から、狙っている現象は

> 手元でホースを保持・操作し、内部を水が流れる柔軟ホースと先端シャワーヘッドが蛇のように振れる／暴れる挙動を抑え、狙った方向へ誘導する

ことであると整理する。

現行 X1-0〜X1-3 は削除しない。これらは **Phase 0: rigid baseline** として凍結し、

> 末端剛体＋定常流水反力だけでは、なぜ勝手に安定してしまうのか

を比較する教材に降格する。

本命は **Phase 1: flexible hose / pipe conveying fluid** とする。

通常の固定ノズルシャワーを TVC と呼ばない。TVC は後段の比較対象であり、Phase 1 の不安定化機構は「柔軟体内部を移動する流体と構造変形の連成」にある。

---

## 1. 学習目標

Side Lab X1 の終了時に、読者が次を説明できる状態を目指す。

1. 末端の噴流反力だけでは、柔軟ホースの暴れを十分に説明できないこと
2. 柔軟梁の固有モードと減衰の意味
3. 内部流速 `U` が構造方程式へ速度依存項・`U^2` 項を持ち込むこと
4. 流量を上げると減衰振動から flutter / 自励振動へ遷移し得ること
5. 「外力を受けて揺れる」のと「流れからエネルギーを受けて自励的に揺れる」の違い
6. 手元境界を動かすことで、柔軟体の複数モードを制御する難しさ
7. P / PD / 状態フィードバックが、剛体1自由度と柔軟多自由度でどう違うか
8. 入力飽和・遅れ・センサ帯域が制御可能性を制約すること
9. TVC・CoP/ZMP・柔軟ホース制御は同一現象ではないが、「実現可能な外力・境界入力で運動を制御する」という制御構造で比較できること

---

## 2. 教材上の位置づけ

本編の番号は変更しない。

推奨導線:

```text
02 倒立振子
  ↓
03 PID / PD
  ↓
Side Lab X1
  ├─ Phase 0: 剛体近似
  └─ Phase 1: 柔軟ホース + 内部流れ
        ↓
    固有モード / flutter / 境界制御
        ↓
04 状態空間 / 05 LQR
        ↓
06 ZMP / 07 LIPM・Capture Point
```

Phase 0 と Phase 1 を並べて、

- 剛体近似では重力＋減衰により自然に落ち着く
- 柔軟ホースでは内部流れとの連成により、条件次第で振動が成長する

という差を最初に見せる。

---

## 3. Phase 0 — rigid baseline

### 3.1 位置づけ

現行 X1-0〜X1-3 の hanging rigid-body model を比較用として保持する。

モデル:

- 手元支点は上
- COM / ヘッドは下
- 水は下向き
- 流水反力は上向き
- ヘッドは単一剛体
- ホースは回転ばね・ダンパ
- quaternion による3D姿勢

このモデルは **「柔軟ホースの本命モデル」ではない**。

### 3.2 Phase 0 で学ぶこと

- 流量 `Q` と運動量流束
- `r × F`
- 重心が支点より下なら重力は復元的
- 剛体＋粘性減衰なら自然に安定しやすい
- 「記事・実演のような蛇行が出ない」こと自体がモデル不足の証拠

### 3.3 UI

現行 3D ページを残し、監査完了後は画面上に

> Phase 0 — rigid approximation  
> このモデルは柔軟ホースの flutter を含まないため、自然に安定しやすい

と常時表示する。

---

## 4. Phase 1 — flexible hose / conveying-fluid model

### 4.1 第一版は 2D

最初から3D柔軟体へ行かない。

まず鉛直面内の横変位 `y(s,t)` を持つ柔軟ホースを実装する。

- `s ∈ [0,L]`: ホース軸方向
- `y(s,t)`: 横変位
- `EI`: 曲げ剛性
- `m_s`: ホース構造の単位長さ質量
- `m_f = rho A_hose`: 内部水の単位長さ質量
- `U = Q / A_hose`: 平均内部流速
- `c`: 構造・材料の有効減衰

連続体の基準式は、符号規約を固定したうえで概念的に

```text
(m_s + m_f) y_tt
+ c y_t
+ EI y_ssss
+ 2 m_f U y_st
+ m_f U^2 y_ss
= f_ext
```

とする。

重要なのは、

- `2 m_f U y_st`: 流れと構造速度の結合
- `m_f U^2 y_ss`: 流速二乗で効く項

を、単なる末端反力とは別に持つことである。

実装時は文献と離散化の符号規約を再確認し、式をそのまま雰囲気で写さない。

### 4.2 離散化

Phase 1 第一版は **Euler–Bernoulli beam FEM** を第一候補とする。

各ノード:

```text
q_i = [y_i, theta_i]
```

2節点要素を 8〜16 要素程度使い、

```text
M q_ddot + C(U) q_dot + K(U) q = f
```

へ落とす。

行列は少なくとも次を分離して保持する。

- `M_struct`: ホース構造質量
- `M_fluid`: 内部水の移動質量
- `K_bend`: 曲げ剛性
- `G_flow(U)`: 速度比例の非対称 / gyroscopic coupling
- `K_flow(U^2)`: 流速二乗で変わる有効剛性・非保存項
- `C_struct`: 構造減衰

最終式を1つの巨大な更新式へ潰さず、各項を診断可能にする。

### 4.3 境界条件

手元 `s=0` はプレイヤーが操作する境界。

第一版:

- 手元横位置 `y(0,t)`
- 手元角度 `theta(0,t)`

を prescribed boundary とする。

先端 `s=L` にはシャワーヘッドの

- 質量
- 回転慣性
- ヘッド形状
- ノズル方向

を tip mass / tip inertia として加える。

内部流れの出口運動量と、連続体式・自由端境界条件との整合を必ず確認する。**同じ follower-force 効果を分布項と末端力で二重計上しない。**

シャワーヘッド内部で流路が曲がる効果を追加する場合は、これは別の明示的な tip load として扱う。

### 4.4 数値積分

柔軟梁は剛体モデルより stiff になるため、semi-implicit Euler 固定では決め打ちしない。

候補:

1. Newmark-beta
2. generalized-alpha
3. 状態空間化 + 安定な固定刻み積分

第一実装では **線形 Phase 1 コアを先に作り、固有値・臨界流速の回帰が通る積分法を採用**する。

描画フレームと物理刻みを分離する。

### 4.5 非線形は後段

最初は small-deflection linear beam とする。

次段階で必要なら:

- 幾何学的非線形
- 大回転
- 3D bending
- torsion
- ホース自己衝突
- 床・浴槽との接触

を追加する。

第一版で CFD / Navier–Stokes / 水滴粒子法へは行かない。

---

## 5. Phase 1 で再現したい現象

### 5.1 Q = 0

初期変位を与えると減衰振動し、最終的に静止する。

これは通常の柔軟梁。

### 5.2 低流量

固有振動数・減衰率が変化するが、まだ摂動は減衰する。

### 5.3 臨界流量付近

特定モードの実効減衰が小さくなり、振幅が長く残る。

### 5.4 臨界流量超過

微小な初期摂動・ノイズから振幅が成長する。

この **「放っておいても揺れが増える」領域**を Phase 1 の成立条件とする。

線形モデルでは振幅が無限に成長し得るため、ゲーム化時には

- 小さな非線形飽和
- 曲率上限
- 破綻判定

のいずれかを導入する。

「見た目のために適当な正弦波を足す」ことは禁止する。

---

## 6. プレイヤー入力

ノズルをジンバルしない。

プレイヤーが操作するのは手元境界。

PC:

- Pointer drag: 手元横位置
- Shift + drag または別軸: 手元角度
- wheel / key: 流量

タブレット:

- Pointer Events
- 2D pad: 手元位置 / 角度
- flow slider

Phase 1 の本質は、**末端ではなく境界から柔軟多自由度系を制御する**こと。

---

## 7. ゲームステージ

### F1 — Dry hose

`Q = 0`。

手で揺らして固有モードと減衰を見る。

### F2 — Low flow

水を流すが安定領域。

Phase 0 の「末端反力だけ」と比較する。

### F3 — Near critical

流量を上げ、振動が減衰しにくくなる領域を体験する。

### F4 — Flutter

臨界流量を越え、自励振動を発生させる。

ここで初めて「何もしないと暴れる」をゲームの中心にする。

### F5 — Manual stabilization

手元位置・角度を人間が操作し、

```text
tip position
tip angle
hose RMS deflection
```

を規定範囲に保つ。

### F6 — P / PD

まず観測量を少数に絞り、境界入力の P / PD を比較する。

単一角度だけでは高次モードを抑えられないケースも残す。

### F7 — State feedback

FEM 状態を低次元 modal coordinates に射影し、LQR などへ接続する。

ここを本編 04 / 05 への橋にする。

### F8 — Disturbance / uncertainty

- 流量急変
- ホース長変更
- 曲げ剛性変更
- 先端質量変更
- センサノイズ
- 制御遅れ

を入れる。

---

## 8. UI / visualization

### 8.1 Phase selector

同一 Side Lab 内で

- Phase 0: rigid baseline
- Phase 1: flexible hose

を明確に切り替える。

デフォルトは Phase 1。

### 8.2 2D flexible view

Phase 1 第一版は 2D を優先する。

常時表示:

- 手元境界
- ホース中心線
- FEM nodes / elements の表示切替
- シャワーヘッド
- 水流方向
- tip position
- tip angle
- 変形倍率
- 流量
- 現在の安定性指標

### 8.3 HUD

最低限:

- `Q [L/min]`
- `U [m/s]`
- tip displacement
- tip angle
- RMS hose deflection
- dominant mode
- estimated growth / decay rate
- hand input
- saturation flag

### 8.4 グラフ

- tip displacement vs time
- modal amplitude vs time
- hand input vs time
- flow rate vs time

発展:

- eigenvalue plane
- flow speed vs modal damping
- mode shape display

---

## 9. 自動テスト

Phase 0 の既存回帰は残す。

Phase 1 では以下を必須にする。

### 9.1 構造系

- `Q = 0`, `c = 0` でエネルギーが大きく破綻しない
- `Q = 0`, `c > 0` で初期変位が減衰する
- tip mass を増やすと第一固有振動数が下がる
- 要素数を増やすと低次固有振動数が収束する

### 9.2 流体構造連成

- `U = 0` で `G_flow` / `K_flow` が消える（filled hose の `M_fluid` は残る）
- `U -> -U` で速度比例項の符号は反転する
- `U^2` 項は流れ方向反転で不変
- 低流量では摂動が減衰する
- 臨界流量近傍で最大実部固有値が 0 付近へ来る
- 臨界流量超過で少なくとも1モードが成長する
- 要素数変更で臨界流量が大きく飛ばない

### 9.3 数値健全性

- NaN / Inf を出さない
- 物理刻み変更で主要挙動が大きく変わらない
- 描画fpsと物理結果を分離する
- outlet momentum effect を二重計上しない

### 9.4 Web

既存 `npm test` へ統合。

Human Visual Audit では、

- 低流量で自然減衰
- 臨界付近で長く振れる
- 高流量で明確に自励振動
- 手元操作で振れ方が変わる
- タブレット Pointer Events

を実画面で確認する。

---

## 10. ファイル構成

Phase 0 は既存ファイルを保持する。

Phase 1 は分離する。

```text
docs/js/shower/
  phase0/
    ...将来必要なら既存 rigid core を移動
  flexible/
    beam-element.js
    assemble.js
    conveying-flow.js
    shower-head.js
    boundary.js
    integrator.js
    eigen.js
    scenarios.js

docs/labs/x1-shower-tvc/
  index.html
  main.js
  scene.js
  flexible-view.js
  flexible-ui.js

scripts/
  check-shower-model.mjs
  check-flexible-hose-model.mjs
```

最初からファイル移動で既存 Phase 0 を壊さない。Phase 1 が安定した後に整理する。

---

## 11. 実装フェーズ

### Phase 0 — rigid baseline ✅

旧 X1-0〜X1-3。

- P0-0 座標・剛体設計 ✅
- P0-1 1軸剛体 core ✅
- P0-2 quaternion 3D rigid body ✅
- P0-3 Three.js visualization ✅
- Human Visual Audit: **比較モデルとして再監査が必要**

Phase 0 はここで機能追加停止。

### H1-0 — flexible model contract / derivation ✅

座標・Hermite補間・乾燥梁の閉形式要素行列・conveying-fluid の Galerkin 体積項・clamped base・tip mass / inertia・Rayleigh damping・Newmark・固有値による臨界流速定義を `docs/js/shower/flexible/README.md` に固定した。

- 2D beam 座標・符号
- `L, EI, m_s, A_hose, rho`
- `Q -> U`
- FEM element DOF
- conveying-fluid element matrices
- base prescribed BC
- tip mass / inertia BC
- damping model
- outlet momentum の扱い
- 臨界流速の定義

を `docs/js/shower/flexible/README.md` に固定する。

**この段階では描画しない。**

### H1-1 — dry flexible beam ✅

- Euler–Bernoulli FEM
- consistent mass / bending stiffness assembly
- clamped-base DOF elimination
- tip mass / tip rotational inertia
- Rayleigh damping
- average-acceleration Newmark (`beta=1/4, gamma=1/2`)
- dry natural-frequency analysis
- analytical cantilever first-mode convergence
- undamped energy conservation / damped energy decay regression

実装は `docs/js/shower/flexible/`、回帰は `scripts/check-flexible-hose-model.mjs` とし、`npm test` に統合する。

### H1-2 — conveying-fluid coupling ✅

- `M_fluid`
- `G_flow(U)`
- `K_flow(U^2)`
- non-symmetric state matrix / complex eigenvalue analysis
- flow-speed sweep + bisection
- critical-flow estimate
- 4 / 6 / 8 element mesh convergence
- below/above-critical time-domain regression
- time-step refinement

現在の教育用既定値では、8要素で

```text
U_cr ~= 9.4808 m/s
Q_cr ~= 16.08 L/min   (inner diameter 6 mm)
Im(lambda_cr) ~= 14.797 rad/s
```

を得た。低流量 `U=6 m/s` では最大実部が負、高流量
`U=12 m/s` では正となり、時刻歴でも前者は減衰、後者は振幅成長を
確認した。値そのものは実物製品の同定値ではなく、モデル・パラメータ依存の
教育用結果である。

ここで Phase 1 の「内部流れで安定性が変わり、flutter に入る」という
最小物理は成立した。次は H1-3 で shower-head / outlet 境界を現象側へ寄せる。

### H1-3 — shower-head boundary model ✅

- eccentric rigid-head 2x2 tip mass / inertia
- bent head geometry
- outlet direction
- control-volume momentum correction `m_dot(v_in-v_out)`
- nozzle lever-arm moment
- small-angle boundary linearization
- straight/equal-area limit reproduces H1-2 exactly
- zero-flow / bend-sign / positive-definite mass regressions
- eigenvalue sweep and 6 / 8 element convergence

現在の教育用35°曲がりヘッドでは、8要素で

```text
U_cr ~= 8.3871 m/s
Q_cr ~= 14.23 L/min
Im(lambda_cr) ~= 14.045 rad/s
```

となった。H1-2 の直管基準 `U_cr ~= 9.4808 m/s` より臨界が低下するが、
これは実物製品の測定値ではなく、現在の教育用 geometry / stiffness /
damping に対するモデル結果である。

`U=6 m/s` では曲がり由来の追加境界荷重が約
`F_y=-0.5821 N`, `M=-0.0821 N m`。直線・等断面へ戻すと追加荷重は0となり、
H1-2 の行列へ厳密に戻る回帰を必須とする。

### H1-4 — 2D interactive visualization ✅（Human Visual Audit 継続中）

- Phase 1 を既定表示、Phase 0 rigid baseline を比較タブ化
- 8要素 FEM の flexible centerline を Hermite 補間で描画
- 固定 hand boundary / bent shower head / outlet water / head reaction
- FEM nodes 表示切替
- 静的平衡形状を破線で重ねて、動的成分と区別
- `Q=8.0 / 14.0 / 18.0 L/min` の低流量・臨界付近・flutter プリセット
- `Q` 連続スライダーと `Qcr ~= 14.23 L/min` マーカー
- tip displacement / hose dynamic RMS の履歴グラフ
- HUD: `Q`, `U`, tip displacement / angle, RMS, 観測成長率, 形状モード近似, sim time
- small-deflection 範囲超過時は描画を飽和させず停止表示
- 物理 `dt=0.002 s` は固定のまま、表示上の sim-time playback を ×1 / ×2 / ×4 で選択
- 初期条件は 8 mm スケールの滑らかな変位 + 一度だけの小さな broadband 速度摂動。連続人工加振は行わない

H1-4 CI では UI プリセットそのものを時刻歴回帰し、

```text
8 L/min:  late/early RMS ~= 0.062  -> 明確に減衰
18 L/min: late/early RMS ~= 2.702  -> 不安定モードが再成長
Qcr ~= 14.23 L/min
```

を確認した。18 L/min は最初に安定モード成分が減衰してから不安定モードが
立ち上がるため、監査画面では既定の ×4 再生で数十秒の物理時刻を短時間に確認する。

公開後 Human Visual Audit では、2D geometry、静的平衡線、head / water /
reaction、低流量→臨界→flutter の見え方、履歴グラフ、Phase切替、PC /
タブレット表示を確認する。

### H1-5 — manual game

- hand position / angle control
- low-flow / near-critical / flutter stages
- success / failure
- scoring
- tablet controls

### H1-6 — feedback control

- P / PD
- modal sensing
- state-space realization
- LQR comparison
- actuator saturation / delay

### H1-7 — theory page / biped bridge

- rigid vs flexible comparison
- garden-hose instability
- eigenvalues and flutter
- boundary control
- TVCとの相違
- CoP/ZMPとの相違と共通点
- 04 / 05 / 06 / 07 への相互リンク

---

## 12. 完了条件

Phase 1 第一版は以下をすべて満たしたら完了。

- [x] Q = 0 の柔軟ホースが妥当な減衰振動をする
- [x] 低流量で安定
- [x] 臨界流量を数値的に推定できる
- [x] 臨界超過で自励振動が再現される
- [x] mesh / dt 変更に対し主要結果が収束する
- [ ] 手元境界入力で振動を変えられる
- [x] Phase 0 と Phase 1 を画面で比較できる
- [ ] PC / タブレットで操作できる
- [ ] 物理量とUI表示が一致する
- [x] `npm test` が通る
- [ ] Human Visual Audit が完了する

---

## 13. 非目標

Phase 1 第一版では行わない。

- Navier–Stokes CFD
- SPH / 水滴粒子法
- ホース断面変形
- 流体圧縮性
- cavitation
- turbulence の直接数値計算
- 3D self-contact
- 実物製品のパラメータ同定

これらがなくても、まず **「剛体モデルでは消えていた流体構造連成による flutter」** を再現できるかを判定する。

---

## 14. 長期拡張

Phase 1 成立後に検討する。

- 3D Cosserat rod
- torsion
- nonlinear beam
- model reduction
- observer / Kalman filter
- MPC
- delayed human control
- system identification
- real shower videoとの定性的比較

最終的な教材導線は、

```text
剛体1自由度
  ↓
Phase 0: rigid shower
  ↓
Phase 1: flexible hose + internal flow
  ↓
固有モード / eigenvalue / flutter
  ↓
境界制御 / state feedback / LQR
  ↓
実現可能入力・飽和
  ↓
TVC（比較）
  ↓
CoP / ZMP / Capture Point
  ↓
Preview / MPC
  ↓
Centroidal Dynamics / Whole-Body QP
```

とする。
