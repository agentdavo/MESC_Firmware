if [ ! -d gcc-arm-none-eabi-9-2019-q4-major ]; then
  tar -xvf gcc-arm-none-eabi-9-2019-q4-major-mac.tar.bz2
fi

#cd ./MESC_RZT2M/cmake/
#sed -i 's:CMAKE_CURRENT_SOURCE_DIR:CMAKE_CURRENT_LIST_DIR:g' 'GeneratedSrc.cmake'
#cd ../../

#mv ./MESC_RZT2M/Config.cmake ./MESC_RZT2M/Config_.cmake
rm -rf ./MESC_RZT2M/Config.cmake

echo "set(CMAKE_FIND_ROOT_PATH \"${CMAKE_CURRENT_LIST_DIR}/../gcc-arm-none-eabi-9-2019-q4-major/bin\")" > ./MESC_RZT2M/Config.cmake