include(${CMAKE_SOURCE_DIR}/tests/macros.cmake)

find_package(Arrow   REQUIRED CONFIG)
find_package(Parquet REQUIRED CONFIG)

add_boost_test(TestDataSeriesCsvImporter
  SRC
  test_main.cpp
  testDataSeriesCsvImporter.cpp
  test_DataSeriesParquetImporter.cpp
  LIBS
  Antares::data-series-csv-importer
  Arrow::arrow_shared
  Parquet::parquet_shared
  test_utils_unit)
