# pc/ — PC側ソフトウェア（Python）

操作GUI (`coco-operator`)・シミュレータ (`coco-sim`)・ビジョン (`coco-vision`) と、それらが共有するライブラリ `coco_link`。
プロジェクト全体の説明はリポジトリ直下の [README.md](../README.md) を参照。

```bash
cd pc
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
ruff check . && pytest
coco-sim --scenario scenarios/demo_formation.yaml
coco-operator
```
