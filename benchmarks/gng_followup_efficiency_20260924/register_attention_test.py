"""重点候補の順序・区間参照回帰テストの比較コピーへの登録。"""
import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--variant', choices=('before', 'ids', 'spans'), required=True)
    args = parser.parse_args()
    path = args.source / 'CMakeLists.txt'
    text = path.read_text()
    assert 'add_executable(attention_lookup_test' not in text
    marker = '  add_executable(node_id_reuse_test'
    assert text.count(marker) == 1
    variant = {'before': 0, 'ids': 1, 'spans': 2}[args.variant]
    addition = f'''  add_executable(attention_lookup_test test/attention_lookup_test.cpp ${{SRC_FILES}})
  target_include_directories(attention_lookup_test PRIVATE src include ${{Boost_INCLUDE_DIRS}})
  target_compile_definitions(attention_lookup_test PRIVATE GNG_VERSION=${{GNG_VERSION}}
    ATTENTION_LOOKUP_VARIANT={variant})
  target_link_libraries(attention_lookup_test PRIVATE OpenSSL::SSL OpenSSL::Crypto ykpiv)
  add_test(NAME attention_lookup_test COMMAND attention_lookup_test)
'''
    path.write_text(text.replace(marker, addition + marker, 1))
    shutil.copy2(Path(__file__).with_name('attention_lookup_test.cpp'), args.source / 'test/attention_lookup_test.cpp')


if __name__ == '__main__':
    main()
