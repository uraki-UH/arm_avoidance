#!/bin/bash

GNG_DIR=$(pwd)
ROS2_WS_DIR=$GNG_DIR/../..
RELEASE_DIR=$GNG_DIR/release
datetime=$(date +"%Y%m%d_%H%M")
OUTPUT_DIR=$RELEASE_DIR/$datetime
AIS_DIR=$OUTPUT_DIR/AiS-GNG
GNG_ARCH=cpu
LIBGNG=gng_$GNG_ARCH

# CmakeListsをリリース用に変更
cp ./CMakes/ais_gng.release ./ais_gng/CMakeLists.txt
cp ./CMakes/ais_gng.package.release ./ais_gng/package.xml
# ヘッダーをコピー
cp -rf ./$LIBGNG/include/ ./ais_gng/libgng/

# 出力folderの作成
mkdir -p $OUTPUT_DIR

# ビルドと署名を行う関数
build_and_sign() {
    # 1. 最初の引数をアーキテクチャ名として保存する
    ARCH=$1
    CMAKE_ARGS=$2
    
    cd $ROS2_WS_DIR
    echo "Building for architecture: $ARCH"
    rm -rf ./build/$LIBGNG ./install/$LIBGNG ./log
    colcon build --symlink-install --packages-select $LIBGNG --cmake-args $CMAKE_ARGS
    openssl dgst -sha256 -sign $GNG_DIR/key/$GNG_ARCH/binary_sign.pem -out ./install/$LIBGNG/lib/lib$LIBGNG.sig ./install/$LIBGNG/lib/lib$LIBGNG.so
    if [ ! -f ./install/$LIBGNG/lib/lib$LIBGNG.sig ]; then
        echo "lib$LIBGNG.sig が存在しません。スクリプトを終了します。"
        rm -rf $OUTPUT_DIR
        exit 1
    fi
    mkdir -p $GNG_DIR/ais_gng/libgng/lib_$ARCH/
    cp -rf ./build/$LIBGNG/lib$LIBGNG.so $GNG_DIR/ais_gng/libgng/lib_$ARCH/lib$LIBGNG.so
    cp -rf ./install/$LIBGNG/lib/lib$LIBGNG.sig $GNG_DIR/ais_gng/libgng/lib_$ARCH/lib$LIBGNG.sig
}

# リリースを作成する関数
make_release(){
    GNG_VERSION=$1
    OUTPUT_NAME=$2

    cd $ROS2_WS_DIR

    # build x86_64
    build_and_sign "x86_64" "-DCMAKE_BUILD_TYPE=Release -DGNG_VERSION=$GNG_VERSION"

    # build aarch64
    build_and_sign "aarch64" "-DCMAKE_BUILD_TYPE=Release -DGNG_VERSION=$GNG_VERSION -DCMAKE_TOOLCHAIN_FILE=$GNG_DIR/toolchain/aarch64.cmake"

    # copy source
    mkdir -p $AIS_DIR
    cp -r $GNG_DIR/ais_gng $AIS_DIR/ais_gng
    cp -r $GNG_DIR/ais_gng_msgs $AIS_DIR/ais_gng_msgs
    cp -r $GNG_DIR/gng_classification $AIS_DIR/gng_classification
    
    # 中に入る
    cd $AIS_DIR/
    # 不要なファイルを削除
    find -name ".vscode" -exec rm -rf {} \;
    find -name ".git" -exec rm -rf {} \;
    find -name ".gitignore" -exec rm -rf {} \;
    find -name "__pycache__" -exec rm -rf {} \;
    find -name ".vscode"
    find -name ".git*"
    cd ..
    # 圧縮
    zip -r $OUTPUT_NAME ./AiS-GNG
    rm -rf ./AiS-GNG/

    # 完了メッセージ
    echo "Release build completed successfully. Output files are located in $OUTPUT_DIR/$OUTPUT_NAME"
}

make_release 0 "AiS-GNG_move.zip" # 移動型パーセプション
# make_release 1 "AiS-GNG_static.zip" # 固定パーセプション
# make_release 2 "AiS-GNG_map_only.zip" # マッピングオンリー

# 戻る
cd $GNG_DIR
cp ./CMakes/ais_gng.debug ./ais_gng/CMakeLists.txt
cp ./CMakes/ais_gng.package.debug ./ais_gng/package.xml