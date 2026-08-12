# -*- coding: utf-8 -*-
"""
本地 pytest 配置。

ROS 2 foxy 携带的 launch_testing / launch_testing_ros pytest 插件的 hook
（pytest_pycollect_makemodule）在 pytest >= 8 上会报
`TypeError: import_path() missing 1 required keyword-only argument: 'consider_namespace_packages'`，
导致 collect 阶段直接失败。本 conftest 用最保险的方式在测试进程里禁用这两个
插件，让 `colcon test --packages-select abcd_task` 能正常跑通。

`launch_testing` 相关能力本包并不使用；纯函数与节点行为都靠标准 pytest 收集。
"""

collect_ignore_glob = []


def pytest_configure(config):
    # 从已加载的插件里剔除 launch_testing 的 pycollect_makemodule hook 影响。
    for pname in ("launch_testing", "launch_testing_ros"):
        try:
            if config.pluginmanager.hasplugin(pname):
                plugin = config.pluginmanager.get_plugin(pname)
                if plugin is not None:
                    config.pluginmanager.unregister(plugin, name=pname)
        except Exception:
            # 插件不存在或 API 差异都不影响 abcd_task 的测试
            pass
