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

### H1-4A — fast-onset calibration ✅

Human Visual Audit で、H1-4 の linear small-deflection model は flutter 自体を
再現できる一方、実演動画に比べて「暴れ始めるまで」が遅い可能性が指摘された。
H1-5 のゲーム化へ進む前に、**時間スケールの差が単なるパラメータ・初期条件・
手元境界の違いで説明できるか**を切り分ける。

目的は実動画への数値フィットではない。記事から流量・EI・ホース質量・
正確な発振開始時刻は取得できないため、現在は

> 水を流してから約 1〜3 秒で、肉眼で明瞭な大振幅運動へ移るケースを
> linear Phase 1 model の範囲内で作れるか

を qualitative target とする。

#### H1-4A-0 — onset metric

最低限、次を同時に記録する。

- `sigma = max Re(lambda)`
- e-fold time `1 / sigma`
- 初期動的 RMS から 3 倍 / 5 倍へ到達する予測時間
- time-domain で RMS が初期値の 3 倍へ到達する時刻
- `|y|max`, `|theta|max` が small-deflection guard へ達する時刻
- 数値発散ではなく固有値成長と整合しているか

「fast onset」は、time-domain で明瞭な成長が 1〜3 s に現れ、かつその前に
small-deflection guard を即時突破しないことを第一判定とする。

#### H1-4A-1 — parameter-only sensitivity

他を既定値に固定した one-at-a-time sweep と、小規模な組合せ sweep を行う。

探索対象:

- `EI`
- Rayleigh damping `alpha_M / beta_K`
- `Q`
- hose length `L`
- shower-head tip mass / inertia

初期の探索 envelope は教育用・感度確認用であり、実物同定値とは呼ばない。

目安:

```text
EI:        0.15 ... 1.2 N m^2
Q:         8 ... 24 L/min
L:         0.8 ... 1.8 m
tip mass:  0.10 ... 0.35 kg
alpha_M:   0 ... 0.15 1/s
beta_K:    0 ... 5e-4 s
```

ここで 1〜3 s onset が得られるなら、どのパラメータが支配的かを記録し、
単に「暴れさせたいから減衰を下げる」調整は禁止する。

#### H1-4A-2 — initial curvature / mode content

parameter-only で不足する場合、初期状態を直線近傍だけに限定しない。

- smooth initial curvature
- tip offset
- broadband modal seed

を分離して試す。

初期条件を大きくするだけで「見た目が最初から暴れている」状態を作るのではなく、

- 不安定モードへの射影が強くなって onset が早く見えるのか
- 固有値成長率そのものが不足しているのか

を区別する。

#### H1-4A-3 — movable hand boundary

次に手元を固定端から prescribed boundary へ拡張する。

```text
q_b(t) = [y_base(t), theta_base(t)]
```

全体行列を free / boundary DOF に partition し、

```text
M_ff qdd_f + C_ff qd_f + K_ff q_f
  = f_f
  - M_fb qdd_b
  - C_fb qd_b
  - K_fb q_b
```

として、手元運動を物理的な境界入力として入れる。

最初の比較入力は、連続ランダム加振ではなく

- 単発横変位 pulse
- 単発角度 pulse
- 実演を模した短い手首切り返し

に限定する。

#### H1-4A-4 — decision gate

結果を次の3分類で固定する。

**A. parameter-only で fast onset**

- linear conveying-fluid model のまま H1-5 へ進む
- H1-4 の既定 scenario を、感度結果に基づく監査用 scenario へ更新する

**B. initial curvature / movable boundary を入れれば fast onset**

- linear model は「実演の速い立ち上がり」を boundary-excited response として
  再現可能
- H1-5 は movable boundary を本体とし、self-excited growth と human excitation
  を UI 上で区別する

**C. どれでも fast onset を再現できない、または 1〜3 s より前に
small-deflection 仮定が破綻**

- H1-5 を延期
- **H1-4B: geometrically nonlinear beam / 2D Cosserat rod** を新設
- 大回転・有限曲率を入れてから再判定する

H1-4A では CFD / SPH / 3D self-contact へは進まない。

#### H1-4A result

CI 感度解析の結論:

```text
parameter-only fast self-excited onset:
  C — current linear small-deflection model is insufficient

movable-boundary fast visible response:
  B — reproducible inside a valid low-flow linear regime
```

主な数値:

```text
18 L/min reference:
  static max displacement ~= 1.079 m
  static max rotation     ~= 98.27 deg
  sigma                   ~= +0.1000 1/s
  predicted 3x time       ~= 10.98 s
  -> static equilibrium itself is outside the linear guard

5 L/min validity probe:
  static max displacement ~= 133.88 mm
  static max rotation     ~= 10.17 deg
  -> inside the current guard

5 L/min + one hand pulse (10 mm, 5 deg):
  visible onset ~= 1.0 s
  max RMS ~= 27 ... 41 mm depending on 0.20 ... 0.50 s pulse
  -> no guard hit
```

探索した parameter-only の growing case では、1〜3 s より速い固有値成長を
作ること自体はできたが、すべて静的平衡が small-deflection guard 外だった。
初期曲率だけでは valid 1〜3 s self-excited onset は得られなかった。

したがって、**手元入力の仕組みは H1-5 へ再利用するが、動画の高流量・
大振幅運動を物理的に再現したと主張する前に H1-4B を挟む。**

詳細は `docs/js/shower/flexible/H1_4A_CALIBRATION.md`。

### H1-4B — geometrically nonlinear hose ✅（Human Visual Audit 継続中）

H1-4A の C 判定を受け、small-deflection extrapolation から
**inextensible planar finite-rotation rod** へ切り替えた。

状態は segment angle を基本自由度とし、各 segment 長を厳密に保ちながら

```text
r_i(theta)
theta_i
theta_dot_i
```

から中心線を再構成する。曲げエネルギーは隣接 segment の角度差だけに依存し、
全体を剛体回転しても人工的な曲げエネルギーを生じない。

実装済み:

1. ✅ dry planar rod の静的 hanging / bent equilibrium
2. ✅ dry large-amplitude transient
3. ✅ tip mass / bent shower-head boundary
4. ✅ finite-angle conveying-flow terms
5. ✅ nonlinear equilibrium + onset sweep
6. ✅ small-amplitude limit と H1-1/H1-2 の比較
7. ✅ 1〜3 s fast-onset 再判定
8. ✅ 2D nonlinear visualization
9. ⏭ nonlinear movable hand boundary は H1-5-0 で導入

主要回帰:

```text
dry small-amplitude:
  analytical f1 = 0.65026 Hz
  32-seg rod f1 = 0.67122 Hz
  relative error = 3.22%

finite-rotation static test:
  2 N lateral tip load
  tip x ~= 0.412 m
  tip angle ~= 29.3 deg
  energy band in dry transient ~= 5.8e-6

nonlinear conveying-flow small-angle limit:
  Ucr(8)  ~= 10.048 m/s
  Ucr(12) ~= 9.836 m/s
  Ucr(16) ~= 9.739 m/s
  H1-2 Hermite reference ~= 9.481 m/s
```

曲がったシャワーヘッドを finite angle のまま接続した 18 L/min では、

```text
tip x ~= -0.31 m
tip y ~= 1.15 m
tip angle ~= -19.5 deg
max Re(lambda) ~= +0.026 /s
```

となり、H1-4 linear の `~98 deg` 静的回転という破綻した extrapolation を
有限回転平衡へ置き換えられた。

fast-onset 再判定:

```text
現基準・30 L/min:
  onset ~= 1.817 s

教育用 fast sensitivity:
  Q = 22 L/min
  EI = 0.25 N m^2
  L = 1.5 m
  alpha_M = 0.02 1/s
  onset ~= 1.583 s
```

fast sensitivity の refinement:

```text
N=10, dt=.001 -> 1.592 s
N=12, dt=.001 -> 1.583 s
N=16, dt=.001 -> 1.585 s
N=12, dt=.002 -> 1.584 s
```

したがって H1-4A で未解決だった

> 流水だけの自己励起で 1〜3 秒スケールの明瞭な立ち上がりを作れるか

について、**有限回転モデルでは YES** となった。

H1-4B の自動回帰:

- ✅ rigid-body rotation で人工的な曲げエネルギー0
- ✅ segment length preservation
- ✅ small-amplitude first-mode convergence
- ✅ gravity / tip load 下の finite-rotation equilibrium
- ✅ zero-flow で非物理的自己励起なし
- ✅ `U -> -U`: Coriolis odd / `U^2` term even
- ✅ flow 増加で安定→flutter
- ✅ nonlinear static continuation
- ✅ onset mesh / dt refinement
- ⏭ nonlinear hand-boundary work diagnostic は H1-5-0

公開UIでは Phase 1 の既定表示を H1-4B nonlinear とし、
旧 H1-4 linear は比較用サブタブへ残す。

Human Visual Audit では、

- finite-rotation centerline
- nonlinear equilibrium の破線
- 18 / Fast 22 / High-flow 30 の見え方
- 1〜3 s onset の体感
- 大振幅時の自動fit
- PC / タブレットの描画性能

を確認する。

### H1-5 — manual boundary control

#### H1-5-0 — nonlinear movable hand boundary ✅（Human Visual Audit 継続中）

H1-4B の有限回転 rod に prescribed hand boundary

```text
q_b(t) = [x_h(t), theta_h(t)]
```

を導入した。自由rod側は境界加速度との質量結合を保持して

```text
M_ff qdd_f
  = Q_f - b_f - M_fb qdd_b
```

を各時刻で解く。

同時に境界拘束反力を

```text
R_b = M_b* qdd + b_b - Q_b
```

から復元し、

```text
P_hand = F_hand * v_hand + M_hand * omega_hand
W_hand = integral(P_hand dt)
```

を診断する。

実装:

- ✅ 手元横位置 prescribed boundary
- ✅ 手元角度 prescribed boundary
- ✅ hand acceleration -> free rod の inertial coupling
- ✅ hand reaction force / reaction moment
- ✅ instantaneous hand power
- ✅ cumulative boundary work
- ✅ single smooth pulse input
- ✅ fixed-hand limit が H1-4B と一致
- ✅ flow 中に hand pulse が nonlinear state を実際に変える
- ✅ browser HUD / hand reaction visualization

保存系の回帰では、

```text
boundary work      = 0.04959504442 J
mechanical Δenergy = 0.04959504440 J
relative error     ~= 4.4e-10
```

となり、境界仕事の符号・反力復元を数値的に固定した。

公開画面では左右 12 mm、角度 ±5 deg、複合 pulse を任意時刻に入力できる。
pulse は smooth bump で元の手元位置・角度へ戻る。

#### H1-5-1 — direct pointer control ✅（Human Visual Audit 継続中）

H1-5-0 の prescribed boundary を、Pointer Events から直接 target 指定できる
interactive boundary control へ拡張した。

操作:

- 横 drag -> hand lateral target
- 縦 drag -> hand angle target
- pointer release -> target保持
- center button -> targetを neutral へ戻す
- mouse / pen / touch を同じ Pointer Events で処理

Pointer target を物理境界へ瞬間移動させず、

```text
Pointer target
  -> rate-limited hand actuator
  -> prescribed boundary
  -> nonlinear hose
```

とする。

アクチュエータ既定制約:

```text
x_h      : +/- 80 mm
theta_h  : +/- 30 deg
|v_h|    : <= 0.45 m/s
|a_h|    : <= 4.0 m/s^2
|omega_h|: <= 2.8 rad/s
|alpha_h|: <= 20 rad/s^2
```

target tracking は critically-damped 形の acceleration command を作り、
速度・加速度・travel を個別に clamp する。

実装:

- ✅ Pointer capture
- ✅ horizontal / vertical drag の2軸 target mapping
- ✅ target clamp
- ✅ actuator velocity / acceleration limit
- ✅ tablet-compatible Pointer Events
- ✅ actual / target の別描画
- ✅ hand x actual / target history
- ✅ hand angle actual / target history
- ✅ actuator saturation HUD
- ✅ Pointer status HUD
- ✅ H1-5-0 pulse input を比較用として残す

自動回帰:

- ✅ half-width drag -> +80 mm target
- ✅ half-height upward drag -> +30 deg target
- ✅ target range clamp
- ✅ position / angle target への収束
- ✅ speed / acceleration upper bound
- ✅ target reversal
- ✅ neutral return
- ✅ actuator trajectory と prescribed boundary endpoint の一致
- ✅ Pointer Events wiring の静的確認

Human Visual Audit では特に、

- PC mouse drag
- tablet touch drag
- actual / target の追従遅れ
- 指を離した後の target保持
- canvas外へdragしても Pointer capture が継続すること
- 速度・加速度 limit が過剰に操作感を鈍らせないこと

を確認する。

#### H1-5-1A — control authority tuning ✅（Human Visual Audit反映）

H1-5-3 の実機操作監査で、

> dragしてもホースへの効きが弱く感じる

という問題が確認された。

原因は plant ではなく input mapping / actuator tuning 側だった。
従来は900 px幅の画面で50 px横dragしても約9 mmしかtargetが動かず、
さらに rate limit により actual hand の立ち上がりも遅かった。

ゲーム用の既定値を次へ更新する。

```text
hand travel:
  x_h      : +/- 120 mm
  theta_h  : +/- 45 deg

actuator:
  |v_h|     <= 0.75 m/s
  |a_h|     <= 8.0 m/s^2
  |omega_h| <= 4.5 rad/s
  |alpha_h| <= 36 rad/s^2

pointer mapping:
  full-width drag  -> 480 mm command span
  full-height drag -> 180 deg command span
  (physical travel limitsでclamp)
```

狙いは「物理を強くする」ことではなく、

```text
short drag
  -> clearly visible target
  -> hand actuator follows within ~0.2 s
  -> prescribed boundary does measurable work on the hose
```

というゲーム操作のcontrol authorityを確保すること。

自動回帰:

- ✅ 60 px desktop dragで lateral target >= 30 mm
- ✅ 60 px vertical dragで angle target >= 15 deg
- ✅ 0.20 s後にactual handがtargetの70%以上へ追従
- ✅ direct-control regressionで lateral motion > 60 mm
- ✅ direct-control regressionで angular motion > 18 deg
- ✅ flowing hose state difference > 0.05

ホース本体の物理式・flow coupling・game score式は変更しない。

#### H1-5-2 — stabilization game core ✅

**この工程はゲームルールだけに限定する。**
3D描画・浴室空間・3Dカメラ・3Dホース表現をここへ混ぜない。

目的:

- low-flow / near-critical / flutter の3ステージ
- stage start / reset
- success / failure
- 制限時間
- RMS target
- tip-angle-error target
- failure envelope
- target 内滞在率
- boundary effort `integral |P_hand| dt`
- actuator saturation time
- 0–1000 の score

判定は2D nonlinear physicsの実測値から行い、描画fpsには依存させない。

基本ルール:

```text
inside target:
  RMS <= RMS_target
  AND
  |tip angle - equilibrium tip angle| <= angle_target

failure:
  RMS > RMS_fail
  OR
  |tip angle error| > angle_fail
  が一定時間継続

success:
  time limitまでfailureせず
  AND target内滞在率 >= stage minimum
```

score は

- target tracking quality
- boundary effort
- actuator saturation time

を用いる。正味仕事 `integral P dt` だけではエネルギーを入れてから抜く操作が
相殺されるので、操作コストには `integral |P| dt` を使う。

実装済み:

- ✅ F2 Low Flow / F3 Near Critical / F4/F5 Flutter
- ✅ stage start / restart
- ✅ time limit
- ✅ RMS target
- ✅ tip-angle-error target
- ✅ hold-time failure envelope
- ✅ target内滞在率
- ✅ `integral |P_hand| dt` effort
- ✅ actuator saturation time
- ✅ 0–1000 score
- ✅ success / failure
- ✅ game中の flow / preset / pulse lock
- ✅ Pointerのみをplayer inputとして維持
- ✅ game logic をphysics/renderから分離
- ✅ pure game-rule regression
- ✅ browser wiring regression

ゲームルールは物理 `dt` ごとに更新し、render fps へ依存しない。

この工程でゲームの数理仕様を固定した。**次は直ちにH1-5-3へ進む。**

#### H1-5-3 — 3D Game View ✅（Human Visual Audit 継続中）

当初構想の **3Dシャワー制御ゲーム** の第一版。

第一版は **2D nonlinear rod physics を3D空間の1平面へ埋め込む**。
見た目とカメラは3Dだが、plantをrenderer側の別アニメへ差し替えない。

座標写像を

```text
2D nonlinear rod [x, y]
    -> 3D gameplay [X=x, Y=handHeight-y, Z=Z_fixed]
```

として固定した。

実装済み:

- ✅ Three.js をローカルvendorから読み込み
- ✅ bathroom floor / wall / grid
- ✅ nonlinear rod nodesから3D hose segmentを毎frame更新
- ✅ 3D shower-head neck / head / nozzle
- ✅ outlet water stream
- ✅ bent-head reaction arrow
- ✅ actual hand boundary
- ✅ hand target
- ✅ nonlinear equilibrium target marker
- ✅ perspective camera
- ✅ Game / Front / Close の固定camera preset
- ✅ H1-5-2 stage / score HUDを共用
- ✅ 3D canvasでもH1-5-1 Pointer操作
- ✅ 3D Gameを既定表示
- ✅ 2D Debugを比較sub-tabとして保持
- ✅ rendererはphysics stateのconsumerのみ
- ✅ 2D -> 3D point / vector mapping回帰
- ✅ 3D / 2D default visibility回帰
- ✅ 両canvasのPointer wiring回帰

H1-5-3では aiming / wall hit 判定まではゲームルールへ入れない。
これは H1-5-4 で追加する。

Human Visual Audit:

- ホースが3D空間で自然な太さ・長さに見える
- shower head と water の向きがphysicsと一致
- hand actual / target が操作と一致
- Pointer操作とcamera UIが衝突しない
- tabletでもゲーム領域が画面から溢れない
- 2D Debugと3D Gameが同じphysics stateを示す
- Game / Front / Close cameraで重要部分が見切れない

#### H1-5-4 — 3D gameplay polish ✅（Human Visual Audit 継続中）

H1-5-3 の3D表示を、**遊べる第一版**として完結させる工程。

この工程では plant physics は変更しない。
難易度・照準・命中率・結果表示・HUD・tablet操作を game layer に追加する。

##### Difficulty

4段階を固定する。

```text
Easy
  physics      : low-flow 12 L/min
  stability    : 広いtarget / failure envelope
  aiming       : 大きいtarget
  authority    : 1.25x

Normal
  physics      : 18 L/min baseline
  stability    : Expertより広い
  aiming       : 中target
  authority    : 1.12x

Expert
  physics      : 18 L/min baseline
  stability    : H1-5-2 near相当
  aiming       : 小target
  authority    : 1.00x

Insane
  physics      : Fast 22 nonlinear case
  stability    : H1-5-2 flutter相当
  aiming       : 最小target
  authority    : 1.00x
```

難易度ごとに以下を1つのconfigへまとめる。

- scenario / preset
- duration
- RMS target / fail
- tip-angle target / fail
- min stability dwell
- aim target radius
- min water-hit fraction
- aim offset
- effort budget
- actuator authority scale

##### Aiming / water hit

現在の authoritative physics は2D nonlinear rodなので、
水命中判定も同じphysics planeで行う。

```text
nozzle origin r_n
outlet unit direction d
target center r_t
target radius R

s = dot(r_t-r_n, d)

s > 0 かつ
distance(r_t, r_n + s d) <= R
なら hit
```

renderer側の見た目だけで別判定を作らない。

target は各difficultyの静的平衡water-rayを基準に
一定距離先へ置き、必要なら法線方向offsetを加える。

計測:

- instantaneous hit
- hit fraction
- mean miss distance
- stability dwell
- boundary effort
- actuator saturation

##### Success / score

success:

```text
time limitまで failure envelope を継続超過しない
AND stability dwell >= minimum
AND water-hit fraction >= minimum
```

score 0--1000:

- stability tracking quality
- aiming quality / hit fraction
- boundary effort
- actuator saturation

を用いる。

##### 3D presentation

- aiming bullseye
- water hit / miss の色フィードバック
- stage intro
- result overlay
- difficulty / score / hit ratio のcompact HUD
- Game / Front / Close camera tuning
- tabletでcanvasとHUDが画面から溢れないこと

sound は必須にしない。

##### Completion gate

実装済み:

- ✅ Easy / Normal / Expert / Insane
- ✅ difficultyごとの physics preset / stability target / aim radius
- ✅ Easy 1.25x / Normal 1.12x authority
- ✅ physics-plane nozzle ray vs circular bullseye hit test
- ✅ finite water-ray range
- ✅ instantaneous hit / hit fraction / aim quality / mean miss distance
- ✅ stability dwell + hit fraction の両方をsuccess条件化
- ✅ 0--1000 scoreへaimingを統合
- ✅ Three.js bullseye
- ✅ hit中はbullseye + water streamをgreen表示
- ✅ stage intro overlay
- ✅ SUCCESS / FAILED result overlay
- ✅ result overlayから即Restart
- ✅ compact tablet HUD
- ✅ 4難易度の基準water rayが初期bullseyeへ届く回帰
- ✅ game-rule / browser / 3D renderer 回帰

Human Visual Auditでは、

- Easyが入門として実際に簡単か
- Normalが標準難易度として成立するか
- Expert / Insaneが理不尽すぎないか
- bullseyeがカメラ3種で見切れないか
- hitのgreen feedbackが即座に読めるか
- result overlay / Restartがtabletで押しやすいか
- game HUDが1画面内に収まるか

を確認する。

ここまでで

> 3D空間で、流水によって暴れるシャワーホースを手元操作で安定化し、
> 狙った場所へ水を当てる

という当初のゲーム体験を第一版完成とする。

#### H1-5-4A — playability calibration ✅（Human Visual Audit 継続中）

Human Visual Audit で次の2点を確認した。

1. Easy / Normal / Expert は **無操作がほぼ最適**になっている
2. Insane は他難易度から急激に難しくなりすぎる

これはphysicsの問題ではなくgame objective / difficulty curveの問題として扱う。

##### Design invariant

H1-5-4A 以降、少なくとも Normal / Expert / Insane では

```text
zero-input baseline
  -> SUCCESSしてはいけない
  -> または gameplay score ceilingを超えてはいけない
```

を自動回帰にする。

「何もしない」が最適になる設計を禁止する。

Easy も入門操作を必須にするが、最初の数秒は照準を静止させ、
Pointer操作を理解する猶予を残す。

##### Moving bullseye schedule

静的平衡ray上へbullseyeを固定するのをやめ、
difficultyごとに **平衡ray法線方向の deterministic target schedule** を持つ。

```text
Easy:
  0--2.5 s   : center
  2.5 s--    : small offset

Normal:
  start      : off-center
  mid-game   : opposite offset

Expert:
  3回以上 target move
  alternating offsets

Insane:
  Fast 22は維持
  target motionはExpertより少なくする
  代わりにflutter stabilizationを主難度とする
```

targetの移動はstep jumpではなく、smooth transitionを使う。

##### Insane calibration

InsaneはFast 22 nonlinear plantを維持する。

ただし Human Visual Audit を受け、

- aim radius
- required hit fraction
- stability dwell
- failure envelope
- authority scale

を成功可能側へ緩和する。

狙いは

```text
zero input -> fail
simple feedback / skilled human -> success possible
```

であり、単なる罰ゲームにはしない。

##### Automated playability regressions

最低限:

- zero-input baseline のtarget schedule追跡
- Easyも最終的には無操作で満点にならない
- Normal / Expert / Insane は無操作SUCCESS禁止
- moving targetがphysical hand travelから到達可能
- target移動速度がactuator authorityから見て不可能でない
- ideal / assisted aim traceでは各difficultyがSUCCESS可能
- Insaneのrequired hit fractionとfailure envelopeを回帰固定

H1-5-4A 最終設定:

```text
Easy
  target: center -> +200 mm
  move:   2.5--3.7 s
  radius: 150 mm
  min hit: 45%
  authority: 1.25x

Normal
  target: +160 -> -160 mm
  radius: 120 mm
  min hit: 50%
  authority: 1.12x

Expert
  target: +130 -> -140 -> +160 -> -120 mm
  radius: 90 mm
  min hit: 55%
  authority: 1.00x

Insane
  Fast 22
  target: +160 -> -160 mm
  radius: 130 mm
  min hit: 32%
  stability dwell: 25%
  fail RMS: 0.55 m
  fail hold: 0.90 s
  authority: 1.18x
```

zero-input pure game-rule regression:

```text
Easy   -> FAILED aim ratio, hit ~= 33.1%
Normal -> FAILED aim ratio, hit ~=  6.1%
Expert -> FAILED aim ratio, hit ~=  9.2%
Insane -> FAILED aim ratio, hit ~=  8.3%
```

actual nonlinear zero-input regression:

```text
Easy   -> FAILED aim ratio, hit ~= 32.8%, stable ~= 100%
Normal -> FAILED aim ratio, hit ~=  6.1%, stable ~= 100%
Expert -> FAILED aim ratio, hit ~=  9.3%, stable ~= 100%
Insane -> FAILED aim ratio, hit ~=  7.4%, stable ~= 26.8%
```

assisted reachable trace は4難易度すべて SUCCESS / score ~= 919。

したがって、

- 無操作SUCCESSは禁止
- Easyも途中から操作必須
- Normal / Expertは追従操作が必須
- InsaneはFast 22を維持しつつ、無操作即死ではなく照準不足で失敗

まで自動回帰で固定した。

H1-5-4A 完了後は H1-6 へ進む。

### H2-0 — true 3D flexible hose（長期拡張）

ゲーム成立の必須条件にはしない。

2D nonlinear rod を3D空間へ埋め込む H1-5-3 とは分けて、

- 3D Cosserat rod
- two-axis bending
- torsion
- 3D whipping
- self-contact / environment contact

を扱う別フェーズとする。

### H1-6 — feedback control

H1-5 までで manual boundary control / gameplay が成立したので、
H1-6 では同じ hand actuator を自動制御器から駆動する。

**重要:** controller が plant を直接書き換えることは禁止する。

```text
sensor
  -> controller
  -> hand target [x_h*, theta_h*]
  -> existing rate-limited hand actuator
  -> prescribed moving boundary
  -> nonlinear hose plant
```

人間操作と自動制御で actuator / saturation / delay を共有し、
比較条件を揃える。

#### H1-6-0 — control contract / sensing ✅

まず観測量を固定する。

P / PD が使える local sensing:

- tip lateral displacement error
- tip lateral velocity
- tip angle error
- tip angular rate

基準は同じ flow 条件の nonlinear static equilibrium。

```text
e_x     = x_tip - x_tip,eq
e_theta = wrap(theta_tip - theta_tip,eq)
```

tip velocity は rod angle / angular-rate と hand boundary velocity から
解析的に計算する。render差分やfps差分から速度推定しない。

P / PD は rod全node角度を見てはいけない。
full-state sensing は H1-6-2 から解禁する。

#### H1-6-1 — P / PD boundary stabilization ✅（Human Visual Audit 継続中）

最初は aiming を分離し、同一初期摂動に対する振動抑制で比較する。

P:

```text
x_h*     = -Kpx e_x
theta_h* = -Kptheta e_theta
```

PD:

```text
x_h*     = -Kpx e_x - Kdx xdot_tip
theta_h* = -Kptheta e_theta - Kdtheta theta_dot_tip
```

出力は必ず H1-5 の hand target clamp / actuator limits を通す。

自動回帰:

- zero error -> zero control target
- sign symmetry
- P は displacement / angle error のみを見る
- PD は velocity sign に対し減衰方向へ働く
- target clamp
- actuator limits を bypass しない
- baseline 18 L/min で peak RMS を増幅しない
- naturally damped baseline の RMS integral 悪化を上限内へ制限
- Fast 22 で P / PD が RMS integral を明確に低減
- PD が P より強く抑制
- Fast 22 でも数値発散しない
- calibrated P / PD は actuator saturation 0 s
- controller OFF で H1-5 と完全互換

UI:

- Control mode: Human / P / PD
- controller target を hand target として可視化
- control sensing / command HUD
- Human モードでは従来 Pointer をそのまま維持

この段階では game aiming score を勝敗比較へ使わない。
まず stabilizer 単体の効果を分離して確認する。

実装済み既定ゲイン:

```text
P:
  Kpx = Kptheta = -0.04

PD:
  Kpx = Kptheta = -0.08
  Kdx = Kdtheta = -0.015 s
```

ここで負符号は式 `u = -K e` に対する値なので、
実際の hand target は tip displacement / velocity と同方向へ追従する。
この plant では手元をtipと逆向きへ押すと relative deformation を増やし、
初回校正では明確に不安定化したため不採用とした。

production-horizon nonlinear regression:

```text
baseline 18 L/min, 5 s
  open RMS integral = 0.01215 m s
  P                 = 0.01518 m s  (1.249x)
  PD                = 0.01472 m s  (1.212x)
  peak RMS           = all about 10.08 mm
  saturation         = 0 s

Fast 22, 3.5 s
  open RMS integral = 0.27106 m s
  P                 = 0.23073 m s  (0.851x, about 14.9% reduction)
  PD                = 0.10641 m s  (0.393x, about 60.7% reduction)
  open peak RMS      = 433.6 mm
  P peak RMS         = 378.7 mm
  PD peak RMS        = 175.3 mm
  saturation         = 0 s
```

baseline 18 は元々自然減衰が非常に強いので、P/PDはpeakを増やさない一方、
integral / effortでは open-loop より不利になる。
これは「制御を入れれば常に得」という誤解を避ける比較結果として保持する。


#### H1-6-2 — full-state realization / state feedback / LQR ✅（Human Visual Audit 継続中）

ここでのみ full rod state を使用可能にする。

H1-6-2 第一実装では、plantを別の線形モデルへ置き換えず、
**既存 nonlinear rod + existing hand actuator の1-step mapを平衡点まわりで数値線形化**
する。

状態は boundary angle の二重保持を避けて

```text
x =
[
  delta theta_1 ... delta theta_{N-1},
  delta omega_1 ... delta omega_{N-1},
  x_hand,
  v_hand,
  theta_hand,
  omega_hand
]
```

とする。rod segment 0 の angle / angular-rate は hand actuator state から復元する。

入力:

```text
u = [x_hand_target, theta_hand_target]
```

したがって control path は Human / P / PD と同じ

```text
state feedback
  -> hand target
  -> existing rate-limited actuator
  -> prescribed boundary
  -> nonlinear rod
```

であり、LQRだけactuatorを bypassしない。

##### H1-6-2A — discrete full-state realization

物理刻み `dt=0.002 s` の nonlinear one-step map

```text
x_{k+1} = F(x_k, u_k)
```

を nonlinear equilibrium `(x,u)=(0,0)` まわりで central finite difference し、

```text
delta x_{k+1} = A_d delta x_k + B_d delta u_k
```

を得る。

必須回帰:

- equilibrium residual が十分小さい
- central-difference epsilon 変更で主要行列成分が大きく飛ばない
- `B_d` の2入力が0でない
- linear one-step prediction が小摂動 nonlinear step と一致
- state encode/decode round trip

##### H1-6-2B — controllability / effective subspace

controllability matrix

```text
C = [B, AB, A^2B, ...]
```

の numerical rank を計算する。

- full rankならそのまま記録
- full rankでなくても「失敗」とはせず、effective controllable rankを明示
- singular-value threshold相当のscale-aware rank判定を使う
- actuatorを含む augmented state と rod-only state の両方を診断する

##### H1-6-2C — discrete LQR

第一版のproduction controllerは discrete LQR とする。

```text
J = sum_k (x_k^T Q x_k + u_k^T R u_k)
u_k = -K x_k
```

DAREを反復で解き、

- `Q`: rod angle / rod rate / actuator state を別weight
- `R`: hand position target / angle target の使用コスト
- outputは既存 hand target clamp を通す
- actual actuator speed / acceleration saturationも既存実装を共有

する。

##### H1-6-2D — continuous-time cross-check

discrete realizationから

```text
A_c ~= (A_d-I)/dt
B_c ~= B_d/dt
```

の小刻み近似を診断用に持つ。

第一版では continuous CARE gainをproductionに使わず、
continuous/discreteの符号・時定数スケール整合を確認する。
厳密continuous CAREは必要なら後続で追加する。

##### H1-6-2E — nonlinear validation / UI

同一初期摂動で

- open
- P
- PD
- LQR

を比較する。

最低限:

- baseline 18 L/min で数値発散しない
- Fast 22 で open-loop より RMS integral を低減
- P / PD と同じ actuator limit
- saturation timeを記録
- small-amplitude linear predictionとnonlinear responseの方向が一致
- controller OFFでH1-5互換

UI:

- Control mode: Human / P / PD / LQR
- full-state norm
- LQR hand target
- controllability rank
- linearization residual
- saturation flag

optional sensor / actuator delay は H1-6-3 または F8 で追加する。

実装結果:

```text
6-segment diagnostic realization
  augmented dimension      = 14
  controllability rank     = 14 / 14
  rod controllability      = 10 / 10
  equilibrium residual     ~= 2.31e-14
  DARE iterations          = 1948

12-segment Fast 22 production realization
  augmented dimension      = 26
  controllability rank     = 25 / 26
  rod controllability      = 22 / 22
  equilibrium residual     ~= 1.78e-14
  DARE iterations          = 1682
  max |K|                  ~= 23.56
```

finite-difference epsilonを0.5x / 2xしても、
6-segment基準のA/B相対差はおおむね `1e-10` オーダーで一致した。

nonlinear closed-loop regression:

```text
baseline 18 L/min, 8-segment, 4 s
  open RMS integral = 0.00808 m s
  LQR               = 0.00473 m s
  ratio             = 0.586
  peak RMS           ~= same initial peak
  saturation         = 0 s

Fast 22, 8-segment, 3.5 s
  open RMS integral = 0.22338 m s
  LQR               = 0.00799 m s
  ratio             = 0.0358
  open peak RMS      = 373.6 mm
  LQR peak RMS       = 9.13 mm
  saturation         = 0 s
```

UI:

- ✅ Human / P / PD / LQR
- ✅ LQR初回選択時のみ nonlinear finite-difference realization + DARE
- ✅ scenario / flowごとにdesign cache
- ✅ full-state norm表示
- ✅ controllability rank表示
- ✅ equilibrium residual表示
- ✅ LQRも既存 hand target clamp / actuatorを共有
- ✅ game中はH1-6-3までHuman固定

12-segment augmented stateは25/26 rankだが、
**rod stateは22/22 full rank**であるため、1次元の弱いaugmented方向を
「ホースを制御不能」と解釈しない。effective controllable subspaceを表示して保持する。

#### H1-6-3 — Human vs Controller ✅（Human Visual Audit 継続中）

同じ H1-5-4A game condition へ

- Human
- P
- PD
- fixed full-state feedback
- LQR

を投入する。

##### H1-6-3A — common aiming outer loop

自動制御器も Human と同じ moving bullseye を観測する。

bullseye と equilibrium nozzle / outlet ray から、既存 hand travel 制約内で
ray miss が最小になる hand reference `[x_ref, theta_ref]` を求める。

角度を先に決めてから横位置を補正すると hand travel clamp に当たるケースがあったため、
最終実装では **hand angle と lateral shift を同時に制約内探索**する。

```text
moving bullseye
  -> constrained geometric aiming reference u_aim
  -> inner stabilizer
  -> existing hand actuator
  -> nonlinear hose
```

P / PD は aim reference から作るrigid reference tipに対する局所誤差を使う。

full-state / LQR は nonlinear calibration の結果、

```text
u = u_aim - alpha K x
```

という二重ループを採用した。

`u_ref - K(x-x_ref)` をそのまま使うと、非ゼロreferenceに対する `+K x_ref` が大きくなり、
元の平衡点まわりで設計したLQRを遠い姿勢へ移植して不安定化したため不採用。

したがって H1-6-2 のLQRは **元のnonlinear equilibriumまわりのinner stabilizer**、
aimingは outer feedforward と役割分担する。

##### H1-6-3B — fixed full-state baseline

比較用 State FB はLQRと同じfull-state gain方向を使うが、

```text
alpha_state = 0.55
alpha_LQR   = 1.00
```

とする。

State FBは最適制御を名乗らず、

> LQR由来のfull-state gainを弱めた固定gain baseline

として扱う。

Human / P / PD / State FB / LQR は全て

- 同じ hand target clamp
- 同じ actuator speed / acceleration limit
- 同じ difficulty authority scale

を通る。

##### H1-6-3C — game comparison metrics / UI

各runで保存する。

- success / failure
- score
- mean RMS
- hit fraction
- stability dwell fraction
- boundary effort `integral |P_hand| dt`
- actuator saturation time

同一difficultyについてsession内の

- attempts
- successes / success rate
- last score
- best score
- last Hit / mean RMS / Effort / Saturation

を比較表へ蓄積する。

UI:

- ✅ game開始前に Human / P / PD / State FB / LQR を選択
- ✅ moving bullseyeは全mode共通
- ✅ controller runも同じdifficulty buttonから開始
- ✅ State FB / LQR のaim feedforwardをHUD表示
- ✅ session comparison table
- ✅ Humanは従来Pointerを維持

##### H1-6-3D — nonlinear game regression

8-segment / dt=0.002 s / 同一初期摂動での代表結果:

```text
Normal
  Open     FAILED  score 649  hit  6.1%  mean RMS  0.93 mm
  P        SUCCESS score 662  hit 58.8%  mean RMS 42.68 mm
  PD       SUCCESS score 765  hit 81.3%  mean RMS 34.22 mm
  State FB FAILED  score 705  hit 87.4%  mean RMS 71.55 mm
  LQR      SUCCESS score 728  hit 81.1%  mean RMS 43.16 mm

Insane / Fast 22
  Open     FAILED  score 239  hit  6.7%  mean RMS 227.42 mm
  P        FAILED  score  37  hit  8.9%  mean RMS 239.08 mm
  PD       FAILED  score   0  hit 17.3%  mean RMS 155.97 mm
  State FB SUCCESS score 790  hit 86.6%  mean RMS  63.42 mm
  LQR      SUCCESS score 795  hit 82.7%  mean RMS  34.45 mm
```

Insane の State FB / LQR は saturation time 0 s。
LQR effortも約0.039 Jで、強いactuatorを使って勝っているわけではない。

ここから、

- Normalでは局所PDでも十分
- Fast 22では局所tip情報だけのP/PDでは不足
- full-state feedbackがflutter抑制とaimingを両立
- LQRはFast 22で最小mean RMSを達成

という教材上の差をそのまま保持する。

自動回帰:

- ✅ centered aim reference -> neutral hand reference
- ✅ constrained aim referenceはhand clamp内
- ✅ P / PD / State FB / LQR 全てexisting actuatorを通る
- ✅ 全automatic modeでopenよりhit fraction改善
- ✅ Normal P / PD / LQR SUCCESS
- ✅ Insane State FB / LQR SUCCESS
- ✅ Insane full-state系は持続saturationなし
- ✅ Human Pointer pathを維持

H1-6-3 では controller専用の強い actuator を禁止する。


### H1-7 — theory page / biped bridge ✅

- ✅ rigid vs flexible comparison
- ✅ garden-hose instability / flutter を固有値で説明
- ✅ prescribed boundary control の式と意味
- ✅ P / PD と full-state feedback の観測量差
- ✅ discrete LQR と shared actuator の位置づけ
- ✅ TVCとの相違と制御構造上の共通点
- ✅ CoP/ZMPとの相違と「実現可能入力」上の共通点
- ✅ LIPM / Capture Point の発散成分との比較
- ✅ 補講 S10 を SPA route に追加
- ✅ Side Lab X1 と 03 / 04 / 05 / 06 / 07 の双方向リンク

理論ページは `docs/theory/S10_flexible_hose_biped_bridge.md`。
Side Lab X1 を二足歩行そのものとして扱わず、別の物理系で
「不安定モード・状態観測・可制御性・入力制約・フィードバック」を体験してから
本編へ戻る橋として位置づける。

### H1-8 — game-first UX cleanup ✅（Human Visual Audit 継続中）

Human Visual Audit で、機能は揃っていても通常プレイの導線が
「実験パネル / 開発監査UI」に埋もれていることを確認した。

修正方針:

- ✅ difficulty button は選択だけにし、押した瞬間には開始しない
- ✅ 明示的な START button を追加
- ✅ START 前は nonlinear physics を pause して静止待機
- ✅ 難易度既定値は Normal
- ✅ Human / Auto: P / PD / State FB / LQR をゲーム開始前に選択
- ✅ Auto は「プレイヤー操作不要、controller の自動プレイを観察」と明記
- ✅ LQR / State FB の設計計算は選択時ではなく START 時へ遅延
- ✅ 通常表示は Stage / Status / Time / Score / Hit に絞る
- ✅ Stable / Effort / 詳細targetは折りたたみ
- ✅ Human vs Controller session history は折りたたみ
- ✅ flow / preset / pulse / FEM / controller diagnostics は「物理・デバッグ設定」へ退避
- ✅ 時系列グラフは「グラフ・解析表示」へ退避
- ✅ H1-4〜H1-6 の実装履歴は「開発履歴・モデル説明」へ退避
- ✅ Phase 0 / linear reference 切替も「モデル比較・教材モード」へ退避
- ✅ UX regression で「difficulty click -> 即start」を禁止

通常プレイで最初に見える操作は

```text
1. 難易度
2. 操作方式
3. START
4. 3D Game
```

のみに寄せる。

---

## 12. 完了条件

Phase 1 第一版は以下をすべて満たしたら完了。

- [x] Q = 0 の柔軟ホースが妥当な減衰振動をする
- [x] 低流量で安定
- [x] 臨界流量を数値的に推定できる
- [x] 臨界超過で自励振動が再現される
- [x] mesh / dt 変更に対し主要結果が収束する
- [x] H1-4A で fast-onset の時間スケール差を切り分ける
- [x] 手元境界入力で振動を変えられる
- [x] Phase 0 と Phase 1 を画面で比較できる
- [ ] PC / タブレットで操作できる
- [ ] 物理量とUI表示が一致する
- [x] H1-7 theory bridge が 03 / 04 / 05 / 06 / 07 と Side Lab X1 を相互接続する
- [x] H1-8 game-first UX で通常プレイと開発UIを分離する
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
