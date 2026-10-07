# Git 運用ルール

## ブランチ戦略（GitHub Flow + feature ブランチ）

| ブランチ | 用途 |
|---|---|
| `main` | 常に動く状態。直接 push しない（PR 経由でマージ） |
| `feature/<内容>` | 機能追加（例 `feature/mode-cargo-transport`） |
| `fix/<内容>` | バグ修正 |
| `docs/<内容>` | 文書のみの変更 |
| `chore/<内容>` | ビルド・CI・依存関係など |
| `firmware/<内容>` | ESP32 ファームウェア |

1. `main` から作業ブランチを切る: `git switch -c feature/xxx main`
2. 小さな単位でコミット（1コミット = 1つの意味のある変更、テストが通る状態）
3. push して Pull Request を作成。CI（lint + test, Mac/Win/Linux）が緑であることを確認
4. レビュー後 `main` へマージ（履歴を残すため merge commit 推奨）

## コミットメッセージ（Conventional Commits）

```
<type>(<scope>): <要約（日本語可, 50字程度）>

<本文: なぜ変更したか、何を変更したか>
```

| type | 意味 |
|---|---|
| `feat` | 新機能 |
| `fix` | バグ修正 |
| `docs` | 文書 |
| `test` | テストの追加・修正 |
| `refactor` | 振る舞いを変えないコード整理 |
| `chore` | ビルド・CI・依存関係 |
| `perf` | 性能改善 |

scope の例: `protocol`, `sim`, `operator`, `vision`, `sysid`, `modes`, `control`, `firmware`

例:
```
feat(modes): 協調物資運搬モードを追加
fix(sim): ドラッグ解除時にロボット速度が残る問題を修正
docs(protocol): telemetry に rx_count を追加
```

## その他

- ログ（`logs/*.mcap`）・仮想環境（`.venv/`）はコミットしない（`.gitignore` 済み）
- 同定結果 `config/robots/robot_XX.yaml` は **コミットする**（チームで共有するため）
- プロトコルを変える PR は、タイトルに `protocol` scope を付け、レビューでファーム担当者に必ず確認してもらう
- リリース時（中間発表・最終発表など）はタグを打つ: `git tag -a v0.1.0 -m "中間発表版"`
