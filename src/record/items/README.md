# Item Library

## 来源

- 导出时间（UTC）：`20260821_025118`
- 来源：Azure Storage `eaia2ddata/item-library`
- 原始导出包：`item_library_full_dump_20260821_025118.zip`，773 个 Blob、56,603,254 bytes，
  SHA256 `5db1484d72af6e706cdab2e1374b09f102f0919851f2dcdc2126d84ce54b9832`
  （**这个 zip 不在仓库里**，见最后一节）
- SQLite：108 items、475 aliases、100 reference images
- 导出侧校验：`integrity_check = ok`；100 张正式参考图全部存在；下载期间源数据未变化

## 这里只留了 db

导出包 773 个文件 / 57 MB，入库的只有 `item-library/db/item_library.db`（2.1 MB）——
**代码只读它**（`pose_svgs`、`pose_footprints_mm` 都在库里）。删掉的部分与理由：

| 目录 | 体积 | 为什么不要 |
|---|---:|---|
| `preview/` | 49 MB | 662 张历史 backfill 预览图，与采集无关 |
| `crops/` | 6.3 MB | 100 张正式参考图，给人看的，代码不读 |
| `blob_manifest.json` | 1.4 MB | 云端 Blob 清单，指向已经不在这里的文件 |
| `SHA256SUMS` | 88 KB | 776 行里 775 行指向已删的文件。它原本校验的是「从 Azure 下载+解压有没有出错」，现在 db 直接从 git 里出来，git 本身就是内容寻址的，这层校验是冗余的 |
| `stats/` | 108 KB | 对方的 usage 快照。我们用自己的 —— cold-tail 排序要的是「**我们**还没采过什么」 |
| `events/` | 32 KB | 对方的事件日志 |
| `config/` | 8 KB | 对方的 Scene v2 配置。我们的布局按自己的可达掩码算 |
| `validation.json` | 0.6 KB | 导出侧校验结果，关键数字已抄到上面「来源」一节 |

留下的 db 的 SHA256 是
`a0a43a19ddf037a5aaef5ee34c83b65a87dc1d89ffd1062b3fee2dfe378630e5`，
和导出包 `SHA256SUMS` 里那一行一致；将来换库时凭它判断是不是同一份。

```bash
sqlite3 item-library/db/item_library.db      # 直接看库
```

## 需要参考图或完整包时

重新从 Azure 导出，**别提交进来**：57 MB 的二进制资产每导出一次就在 git 历史里永久多
一份，而其中 96% 是采集用不到的图。
