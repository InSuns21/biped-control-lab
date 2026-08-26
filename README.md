# Biped Control Lab

二足歩行ロボットが「倒れずに立つ・歩く」ための制御を、**理論（Markdown）とブラウザ実験（HTML + JavaScript）を往復しながら学ぶ**教材です。

第1版では、いきなり多関節ヒューマノイドへ行かず、制御の本質が見える 2D 簡略モデルに絞ります。

## 第1版の学習経路

1. 二足立位はなぜ難しいのか
2. 倒立振子モデル
3. PID / PD フィードバック
4. 状態空間表現と状態フィードバック
5. LQR の考え方
6. 支持多角形と ZMP
7. LIPM とステップ位置制御

## 実験

- Lab 1: 無制御の倒立振子 — 数度の傾きが増幅して転倒する
- Lab 2: PD 制御 — ゲインと外乱を変えて安定化する
- Lab 3: 支持多角形 — 両足 / 片足で安定領域がどう変わるか見る
- Lab 4: LIPM 歩行 — ZMP 制御と capture point に基づくステップ回復を見る

## ローカル起動

静的サイトなので、`docs/` を HTTP サーバーで配信すれば動きます。

```bash
python -m http.server 8000 -d docs
```

その後 `http://localhost:8000/` を開いてください。

## 品質チェック

```bash
npm install
npm test
```

CI では以下を検査します。

- JavaScript 構文
- Markdown lint
- KaTeX 数式構文
- ローカル静的サイトのリンク切れ
- GitHub Pages 用 artifact の生成

## GitHub Pages

`.github/workflows/pages.yml` は `docs/` をそのまま GitHub Pages にデプロイします。新規リポジトリ作成後、Repository Settings → Pages → Source を **GitHub Actions** に設定してください。

## モデル化上の注意

本教材のシミュレーションは教育目的です。実機のヒューマノイドでは、3次元多リンク剛体、接触・摩擦、関節トルク制約、センサノイズ、状態推定、遅延、全身制御などが追加されます。

## 参考文献

- Miomir Vukobratović and Branislav Borovac, “Zero-Moment Point — Thirty Five Years of Its Life”, *International Journal of Humanoid Robotics*, 2004.
- Shuuji Kajita et al., *Introduction to Humanoid Robotics*, Springer, 2014.
- Jerry Pratt et al., “Capture Point: A Step toward Humanoid Push Recovery”, Humanoids 2006.
- Katsuhiko Ogata, *Modern Control Engineering*.

## License

教材として再利用しやすいよう、公開時に用途に合うライセンス（例: MIT）を設定してください。
