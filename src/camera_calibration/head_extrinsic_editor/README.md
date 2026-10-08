# 头相机外参手调

```bash
python3 src/camera_calibration/head_extrinsic_editor/server.py
```

打开 `http://127.0.0.1:8231`。输入框对应
`camera_calibration/config/calibration.yaml` 中
`urdf_overrides.d435_joint.xyz/rpy`，单位分别为米和弧度。输入不限制小数位；每次变化
都会在当前照片上重新渲染轮廓，不修改 YAML。

“拍新照片”只在进程内存中保留当前原图、0.5 秒关节角、内参、光学段 TF 和实时 neck
变换；不会写盘。再次拍照会销毁上一张，刷新页面后前端也不会恢复当前照片。

预览使用页面输入的 XYZ/RPY，不需要重启控制栈。机器人拍照时必须静止；左侧会显示
采样窗口最大关节变化。