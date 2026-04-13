# 目前はヘッダーのみですが、将来的な拡張と一貫性のためにライブラリ定義を作成
add_library(description_lib INTERFACE)
target_include_directories(description_lib INTERFACE .)
