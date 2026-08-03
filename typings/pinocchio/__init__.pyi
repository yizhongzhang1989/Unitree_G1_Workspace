"""Pinocchio 的类型 stub（占位）。

Pinocchio 的 Python API 全部来自 Boost.Python 编译的 C 扩展
``pinocchio_pywrap_default.cpython-310-*.so``，包的 ``__init__.py`` 只是
``from .pinocchio_pywrap_default import *`` 转发一层。Pylance 读不出 ``.so``
里的符号，官方也没有随包发 ``.pyi`` / ``py.typed``，所以
``pin.forwardKinematics``、``pin.SE3`` 之类会被报成
“不是模块 pinocchio 的已知属性”——这是纯粹的假阳性，运行时全都存在。

这里用 PEP 484 的模块级 ``__getattr__``：任何未显式声明的属性都按 ``Any``
处理。比在 settings.json 里全局关 ``reportAttributeAccessIssue`` 精准得多，
其它模块的同类错误仍然会报出来。stub 只给类型检查器看，运行时无影响。

想要真正的补全，可以 ``pip install mypy`` 后
``stubgen -m pinocchio -o typings`` 覆盖本文件。
"""

from typing import Any

def __getattr__(name: str) -> Any: ...
