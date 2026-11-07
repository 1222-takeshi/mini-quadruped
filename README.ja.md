# mini-quadruped

ミニ四脚歩行ロボットです。MG90S級のメタルギアマイクロサーボ8個（各脚に
股ピッチ＋膝）を、Arduino Nano（互換機）が PCA9685 経由で駆動します。
まずはトロット歩容から。学生時代の可変構造6脚ロボット研究のミニ版という
位置付けです。

English README: [README.md](README.md)

## ハードウェア

| 役割 | 部品 |
|---|---|
| 脚 | MG90S級サーボ ×8（各脚に股ピッチ＋膝） |
| PWMドライバ | PCA9685 16ch（I2C） |
| 制御 | Arduino Nano（互換機） |
| フレーム | 3Dプリント（FLASHFORGE Adventurer 3、150×150×150 mm） |

## リポジトリ構成

- `software/leg_ik.*` - 2リンク脚の逆運動学・順運動学（C++17）
- `software/gait.*` - トロット歩容ジェネレータ（対角ペア、遊脚期は楕円
  軌跡、支持期は直線）
- `software/tests/` - 検証バイナリ。IK往復一致、足先軌跡の連続性、
  サーボ角度範囲、周期の連続性を確認。`make test` で実行
- `software/firmware/` - Arduino Nano用ファームウェア骨子。C++参照実装と
  歩容パラメータを共有
- `docs/PLAN.md` - 設計計画、`ROADMAP.md` - 開発ロードマップ
- `CONTRIBUTING.md` - 開発プロセス（ロードマップ → Issue → TDD →
  小さいPR → レビュー → マージ）

## 状態

運動学と歩容は実装済みで、数値検証は全てパスしています（参照値と一致）。
サーボ到着待ちで、次はフレーム設計とファームウェアの立ち上げです。
