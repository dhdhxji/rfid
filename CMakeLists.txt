cmake_minimum_required(VERSION 3.10)

project(rfid)

add_library(${PROJECT_NAME} STATIC "src/MFRC522.c")
target_include_directories(${PROJECT_NAME} PUBLIC "src")