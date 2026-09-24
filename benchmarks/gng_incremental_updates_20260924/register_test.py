"""フレーム差分更新と固定dense参照の回帰テスト登録。"""
from pathlib import Path
import sys
import shutil
source = Path(sys.argv[1])
shutil.copy2(Path(__file__).with_name('incremental_updates_test.cpp'), source/'test/incremental_updates_test.cpp')
path = source/'CMakeLists.txt'
text = path.read_text()
marker = '  add_executable(search_cache_test '
assert text.count(marker) == 1
target = """  add_executable(incremental_updates_test test/incremental_updates_test.cpp test/edge_dense_reference/cugng.cpp
    src/cpu/cugng.cpp src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(incremental_updates_test PRIVATE src include)
  target_compile_definitions(incremental_updates_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME incremental_updates_test COMMAND incremental_updates_test)
"""
path.write_text(text.replace(marker, target + marker, 1))
