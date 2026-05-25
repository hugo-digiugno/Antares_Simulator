// Copyright 2007-2026, RTE (https://www.rte-france.com)
// SPDX-License-Identifier: MPL-2.0

#define WIN32_LEAN_AND_MEAN
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unit_test_utils.h>

#include <boost/test/unit_test.hpp>

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>

#include <antares/io/inputs/data-series-csv-importer/DataSeriesRepoImporter.h>

using namespace std;
using namespace Antares::IO::Inputs::DataSeriesCsvImporter;
using namespace Antares::Optimisation::LinearProblemDataImpl;

struct ParquetCreationFixture
{
    filesystem::path temp_path;

    ParquetCreationFixture()
    {
        temp_path = filesystem::temp_directory_path() / std::tmpnam(nullptr);
        filesystem::create_directories(temp_path);
    }

    ~ParquetCreationFixture()
    {
        if (filesystem::exists(temp_path))
        {
            filesystem::remove_all(temp_path);
        }
    }

    // Write a column-major double table to a .parquet file.
    // cols[c] holds all row values for scenario column c.
    filesystem::path writeParquet(const string& stem,
                                  const vector<vector<double>>& cols)
    {
        int numCols = static_cast<int>(cols.size());
        int64_t numRows = numCols > 0 ? static_cast<int64_t>(cols[0].size()) : 0;

        vector<shared_ptr<arrow::Field>> fields;
        vector<shared_ptr<arrow::Array>> arrays;

        for (int c = 0; c < numCols; ++c)
        {
            fields.push_back(arrow::field("col" + to_string(c), arrow::float64()));
            arrow::DoubleBuilder builder;
            BOOST_REQUIRE(builder.AppendValues(cols[c]).ok());
            shared_ptr<arrow::Array> arr;
            BOOST_REQUIRE(builder.Finish(&arr).ok());
            arrays.push_back(arr);
        }

        auto table = arrow::Table::Make(arrow::schema(fields), arrays);
        auto path = temp_path / (stem + ".parquet");
        auto maybeOut = arrow::io::FileOutputStream::Open(path.string());
        BOOST_REQUIRE(maybeOut.ok());
        BOOST_REQUIRE(
          parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), *maybeOut, numRows)
            .ok());
        return path;
    }
};

BOOST_FIXTURE_TEST_SUITE(_DataSeriesParquetImport_, ParquetCreationFixture)

BOOST_AUTO_TEST_CASE(parquet_single_column)
{
    writeParquet("series", {{10.0, 20.0, 30.0}});
    auto repo = DataSeriesRepoImporter::importFromDirectory(temp_path);
    BOOST_CHECK_EQUAL(repo.getDataSeries("series").getData(1, 0), 10.0);
    BOOST_CHECK_EQUAL(repo.getDataSeries("series").getData(1, 1), 20.0);
    BOOST_CHECK_EQUAL(repo.getDataSeries("series").getData(1, 2), 30.0);
}

BOOST_AUTO_TEST_CASE(parquet_multi_column)
{
    writeParquet("multi",
                 {{1.1, 2.2, 3.3, 4.4}, {5.5, 6.6, 7.7, 8.8}, {9.9, 10.1, 11.2, 12.3}});
    auto repo = DataSeriesRepoImporter::importFromDirectory(temp_path);
    BOOST_CHECK_EQUAL(repo.getDataSeries("multi").getData(1, 0), 1.1);
    BOOST_CHECK_EQUAL(repo.getDataSeries("multi").getData(1, 2), 3.3);
    BOOST_CHECK_EQUAL(repo.getDataSeries("multi").getData(2, 1), 6.6);
    BOOST_CHECK_EQUAL(repo.getDataSeries("multi").getData(2, 3), 8.8);
    BOOST_CHECK_EQUAL(repo.getDataSeries("multi").getData(3, 0), 9.9);
    BOOST_CHECK_EQUAL(repo.getDataSeries("multi").getData(3, 3), 12.3);
}

BOOST_AUTO_TEST_CASE(parquet_conflicts_with_csv)
{
    writeParquet("data", {{1.0, 2.0}});
    ofstream(temp_path / "data.csv") << "3.0\n4.0";
    BOOST_CHECK_EXCEPTION(
      DataSeriesRepoImporter::importFromDirectory(temp_path),
      DataSeriesRepository::DataSeriesAlreadyExists,
      checkMessage("Data series repo : data series 'data' already exists"));
}

BOOST_AUTO_TEST_CASE(parquet_null_value)
{
    arrow::DoubleBuilder builder;
    BOOST_REQUIRE(builder.Append(1.0).ok());
    BOOST_REQUIRE(builder.AppendNull().ok());
    BOOST_REQUIRE(builder.Append(3.0).ok());
    shared_ptr<arrow::Array> arr;
    BOOST_REQUIRE(builder.Finish(&arr).ok());

    auto table = arrow::Table::Make(arrow::schema({arrow::field("col0", arrow::float64())}), {arr});
    auto path = temp_path / "nulldata.parquet";
    auto maybeOut = arrow::io::FileOutputStream::Open(path.string());
    BOOST_REQUIRE(maybeOut.ok());
    BOOST_REQUIRE(
      parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), *maybeOut, 3).ok());

    BOOST_CHECK_EXCEPTION(DataSeriesRepoImporter::importFromDirectory(temp_path),
                          Antares::IO::Inputs::InputError,
                          containsMessage("Null"));
}

BOOST_AUTO_TEST_CASE(parquet_non_numeric_column)
{
    arrow::StringBuilder builder;
    BOOST_REQUIRE(builder.Append("hello").ok());
    BOOST_REQUIRE(builder.Append("world").ok());
    shared_ptr<arrow::Array> arr;
    BOOST_REQUIRE(builder.Finish(&arr).ok());

    auto table = arrow::Table::Make(arrow::schema({arrow::field("col0", arrow::utf8())}), {arr});
    auto path = temp_path / "strdata.parquet";
    auto maybeOut = arrow::io::FileOutputStream::Open(path.string());
    BOOST_REQUIRE(maybeOut.ok());
    BOOST_REQUIRE(
      parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), *maybeOut, 2).ok());

    BOOST_CHECK_EXCEPTION(DataSeriesRepoImporter::importFromDirectory(temp_path),
                          Antares::IO::Inputs::InputError,
                          containsMessage("cannot be cast to double"));
}

BOOST_AUTO_TEST_CASE(parquet_float32_cast_to_double)
{
    arrow::FloatBuilder builder;
    BOOST_REQUIRE(builder.Append(1.5f).ok());
    BOOST_REQUIRE(builder.Append(2.5f).ok());
    BOOST_REQUIRE(builder.Append(3.5f).ok());
    shared_ptr<arrow::Array> arr;
    BOOST_REQUIRE(builder.Finish(&arr).ok());

    auto table = arrow::Table::Make(arrow::schema({arrow::field("col0", arrow::float32())}), {arr});
    auto path = temp_path / "float32data.parquet";
    auto maybeOut = arrow::io::FileOutputStream::Open(path.string());
    BOOST_REQUIRE(maybeOut.ok());
    BOOST_REQUIRE(
      parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), *maybeOut, 3).ok());

    auto repo = DataSeriesRepoImporter::importFromDirectory(temp_path);
    BOOST_CHECK_EQUAL(repo.getDataSeries("float32data").getData(1, 0), 1.5);
    BOOST_CHECK_EQUAL(repo.getDataSeries("float32data").getData(1, 1), 2.5);
    BOOST_CHECK_EQUAL(repo.getDataSeries("float32data").getData(1, 2), 3.5);
}

BOOST_AUTO_TEST_SUITE_END()
