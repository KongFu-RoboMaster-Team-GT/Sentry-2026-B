#!/bin/zsh

input_dir="."
output_dir="."

mkdir -p "$output_dir"

for f in "$input_dir"/*.stl(.N); do
    filename=${f:t:r}   # zsh 用 :t 取 basename, :r 去掉扩展名
    assimp export "$f" "$output_dir/$filename.dae"
done

