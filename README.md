# Biped Control Lab

二足歩行ロボットが「倒れずに立つ・歩く」ための制御を、**理論（Markdown）とブラウザ実験（HTML + JavaScript）を往復しながら学ぶ**教材です。

第1版では、いきなり多関節ヒューマノイドへ行かず、制御の本質が見える 2D 簡略モデルに絞ります。

公開サイト: [https://insuns21.github.io/biped-control-lab/](https://insuns21.github.io/biped-control-lab/)

## 対象読者

本文は、大学1年程度の次の内容を履修済みと想定します。

- 1変数の微分・積分
- 初歩的な Taylor 展開・多変数微分
- ベクトル・行列・2次行列の固有値
- Newton の運動方程式
- 力、トルク、仕事、エネルギー

一方、**制御工学・ロボティクスは未習でよい**構成にします。

状態空間、可制御性、LQR、Riccati 方程式、ZMP、LIPM、Capture Point などは、公式を前提知識として置かず、本編または「補講」で導入します。

## 教材の読み方

各理論章は原則として

> 直感 → モデル化の仮定 → 導出 → 数式の意味 → 安定性 / 制御上の意味 → Lab で確認

の順で読めるようにします。

発展的な内容は「補講」に分離しているので、最初は読み飛ばして本筋だけ進めても構いません。

## 第1版の学習経路

1. 二足立位はなぜ難しいのか
2. 倒立振子モデルと線形化
3. PID / PD フィードバックと安定条件
4. 状態空間表現、固有値、可制御性
5. LQR と Riccati 方程式の意味
6. 支持多角形と ZMP の導出
7. LIPM、Capture Point、ステップ位置制御

## 独立補講

本編を読んでいて理論的な前提を補いたいときは、次の補講を必要なところだけ参照します。

- S01: 常微分方程式と指数モード — 時定数、特性方程式、連続時間 / 離散時間
- S02: 固有値・固有ベクトルを制御の言葉で読む — モード分解、閉ループ固有値
- S03: Lyapunov安定性 — 解を全部求めずに安定性を調べる
- S04: 状態推定とKalman filter — オブザーバ、可観測性、予測と観測更新
- S05: Lagrange法と多リンクロボット力学 — $M(q)\ddot{q}+C(q,\dot{q})\dot{q}+g(q)$ から全身制御へ
- S06: 接触・摩擦・CoP制約 — 摩擦円錐、contact wrench、ZMP条件の限界

補講は本編の必須順序ではありません。たとえば第04章の固有値が抽象的なら S01/S02、第06章の接触条件を深掘りしたければ S06、実機への橋渡しを見たければ S04〜S06 を読む構成です。

## 実験

- Lab 1: 無制御の倒立振子 — 数度の傾きが増幅して転倒する
- Lab 2: PD 制御 — ゲインと外乱を変えて安定化する
- Lab 3: 支持多角形 — 両足 / 片足で安定領域がどう変わるか見る
- Lab 4: LIPM 歩行 — ZMP 制御と Capture Point に基づくステップ回復を見る

## AI / 人間向け執筆規約

このリポジトリを編集する場合、ルートの [`AGENTS.md`](AGENTS.md) を執筆・編集規約の正本とします。

特に理論ノートでは

- 新しい専門語・記号を初出で定義する
- モデル化の仮定を明示する
- 導出を飛ばして公式だけ置かない
- 厳密式と近似式を区別する
- 「安定」の意味と成立条件を明示する
- 発展事項は「補講」に分離する
- 理論、Lab、図、SPA ルーティングを同時に確認する

ことを必須とします。

Claude Code 用の `CLAUDE.md`、GitHub Copilot 用の `.github/copilot-instructions.md` も `AGENTS.md` を正本として参照します。

## ローカル起動

依存関係を取得してテストすると、公開用の Marked / KaTeX が `docs/vendor/` に生成されます。

```bash
npm install
npm test
python -m http.server 8000 -d docs
```

その後 `http://localhost:8000/` を開いてください。

## 品質チェック

`npm test` では以下を検査します。

- 公開用 Marked / KaTeX アセットの生成
- JavaScript 構文
- 制御モデル回帰テスト
- KaTeX 数式構文
- Markdown lint
- ローカルリンクと公開アセットの存在

`docs/vendor/` は npm 依存から生成される第三者配布物なので、教材本文の Markdown lint / リンク検査の対象外です。公開に必要なファイルの存在と実行可否は専用の Web runtime 検査で確認します。

## GitHub Pages

`.github/workflows/pages.yml` は `npm test` で公開アセットを生成・検証した後、`docs/` を GitHub Pages にデプロイします。Repository Settings → Pages → Source は **GitHub Actions** を使用します。

## モデル化上の注意

本教材のシミュレーションは教育目的です。実機のヒューマノイドでは、3次元多リンク剛体、接触・摩擦、関節トルク制約、センサノイズ、状態推定、遅延、全身制御などが追加されます。

理論章で「安定」と述べる場合も、特に断りがなければ簡略化したモデル・平衡点近傍での議論です。実機全域での安定性を保証するものではありません。

## 参考文献

- Miomir Vukobratović and Branislav Borovac, “Zero-Moment Point — Thirty Five Years of Its Life”, *International Journal of Humanoid Robotics*, 2004.
- Shuuji Kajita et al., *Introduction to Humanoid Robotics*, Springer, 2014.
- Jerry Pratt et al., “Capture Point: A Step toward Humanoid Push Recovery”, Humanoids 2006.
- Katsuhiko Ogata, *Modern Control Engineering*.

## License

教材として再利用しやすいよう、公開時に用途に合うライセンス（例: MIT）を設定してください。
