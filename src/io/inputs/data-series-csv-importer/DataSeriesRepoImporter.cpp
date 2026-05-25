// Copyright 2007-2026, RTE (https://www.rte-france.com)
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <charconv>
#include <ranges>
#include <sstream>
#include <string>
#include <vector>

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/iostreams/stream.hpp>

#include <arrow/api.h>
#include <arrow/compute/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

#include <antares/io/inputs/data-series-csv-importer/DataSeriesRepoImporter.h>
#include <antares/optimisation/linear-problem-data-impl/timeSeriesSet.h>

namespace fs = std::filesystem;

namespace Antares::IO::Inputs
{
struct InputError: std::runtime_error
{
    using std::runtime_error::runtime_error;
};
} // namespace Antares::IO::Inputs

namespace Antares::IO::Inputs::DataSeriesCsvImporter
{
using namespace Optimisation::LinearProblemDataImpl;
using Antares::IO::Inputs::InputError;

inline const char* ParseOneDouble(const char* ptr,
                                  const char* end,
                                  double& value,
                                  const std::string& errorMessagePrefix = "")
{
    auto [p, ec] = std::from_chars(ptr, end, value);
    if (ec == std::errc::invalid_argument)
    {
        throw std::invalid_argument(errorMessagePrefix + ": \"" + *p + "\" is not a number");
    }
    return p;
}

inline void SkipWhiteSpaceAndSeparator(const char*& ptr, const char* last, char sep)
{
    // Skip leading whitespace and separators
    while (ptr < last && (*ptr == sep || *ptr == ' ' || *ptr == '\t'))
    {
        ++ptr;
    }
}

static bool ParseRow(const char* first,
                     const char* last,
                     std::vector<std::vector<double>>& columns,
                     unsigned rowIndex,
                     unsigned rowCount,
                     char sep = ' ',
                     const std::string& errorMessagePrefix = "")
{
    const char* ptr = first;
    unsigned colIndex = 0;
    while (ptr < last)
    {
        SkipWhiteSpaceAndSeparator(ptr, last, sep);

        // If we've reached the end, break
        if (ptr >= last)
        {
            break;
        }

        double val = 0.;
        const char* next = ParseOneDouble(ptr, last, val, errorMessagePrefix);
        // Check if we parsed anything
        if (next == ptr)
        {
            // Skip invalid characters and try again
            ++ptr;
            continue;
        }
        // initialize columns on first row
        if (rowIndex == 0)
        {
            std::vector<double> col(rowCount);
            col[rowIndex] = val;
            columns.emplace_back(std::move(col));
            ptr = next;
            ++colIndex;
        }
        else
        {
            if (colIndex < columns.size())
            {
                columns[colIndex][rowIndex] = val;
                ptr = next;
                ++colIndex;
            }
            else
            {
                std::ostringstream oss;
                oss << errorMessagePrefix << ": row (" << rowIndex << ") has more columns ("
                    << colIndex + 1 << ") than the expected (" << columns.front().size() << ").";
                throw std::invalid_argument(oss.str());
            }
        }
    }
    if (rowIndex != 0 && colIndex != columns.size())
    {
        std::ostringstream oss;
        oss << errorMessagePrefix << ": row (" << rowIndex << ") has less columns (" << colIndex
            << ") than the expected (" << columns.front().size() << ").";
        throw std::invalid_argument(oss.str());
    }
    return colIndex != 0;
}

static std::vector<std::vector<double>> readCSV(const std::filesystem::path& filename, char sep)
{
    // Check file size first
    std::error_code ec;
    auto sz = std::filesystem::file_size(filename, ec);
    if (ec)
    {
        throw std::invalid_argument("Error reading CSV file( " + filename.string()
                                    + "):" + ec.message());
    }
    if (sz == 0)
    {
        return {}; // empty or inaccessible
    }
    const auto& fileName = filename.string();
    boost::iostreams::mapped_file_source file(fileName);

    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + fileName);
    }

    std::vector<std::vector<double>> columns;
    const char* start = file.data();
    const char* end = start + file.size();
    unsigned lineCount = std::count(start, end, '\n') + 1;
    unsigned lineNumber = 0;

    while (start < end)
    {
        const char* endLine = static_cast<const char*>(memchr(start, '\n', end - start));
        if (!endLine)
        {
            endLine = end;
        }
        // Handle Windows line endings
        size_t lineLen = endLine - start;
        if (lineLen > 0 && start[lineLen - 1] == '\r')
        {
            lineLen--;
        }

        if (ParseRow(start, start + lineLen, columns, lineNumber, lineCount, sep, fileName))
        {
            ++lineNumber;
        }
        start = endLine + 1;
    }
    if (lineCount != lineNumber)
    {
        for (auto& col: columns)
        {
            col.resize(lineNumber);
        }
    }
    return columns;
}

static std::vector<std::vector<double>> readParquet(const std::filesystem::path& filename)
{
    auto maybeFile = arrow::io::ReadableFile::Open(filename.string());
    if (!maybeFile.ok())
    {
        throw InputError("Failed to open Parquet file '" + filename.string()
                         + "': " + maybeFile.status().ToString());
    }

    std::unique_ptr<parquet::arrow::FileReader> reader;
    arrow::Status status = parquet::arrow::OpenFile(*maybeFile,
                                                    arrow::default_memory_pool(),
                                                    &reader);
    if (!status.ok())
    {
        throw InputError("Failed to open Parquet reader for '" + filename.string()
                         + "': " + status.ToString());
    }

    std::shared_ptr<arrow::Table> table;
    status = reader->ReadTable(&table);
    if (!status.ok())
    {
        throw InputError("Failed to read Parquet table from '" + filename.string()
                         + "': " + status.ToString());
    }

    const int64_t numRows = table->num_rows();
    const int numCols = table->num_columns();

    std::vector<std::vector<double>> columns(numCols, std::vector<double>(numRows));

    for (int colIdx = 0; colIdx < numCols; ++colIdx)
    {
        auto column = table->column(colIdx);
        const std::string fieldName = table->schema()->field(colIdx)->name();

        if (column->type()->id() != arrow::Type::DOUBLE)
        {
            if (!arrow::is_numeric(column->type()->id()))
            {
                throw InputError("Column '" + fieldName
                                 + "' cannot be cast to double in Parquet file: "
                                 + filename.string());
            }
            auto castResult = arrow::compute::Cast(*column,
                                                   arrow::float64(),
                                                   arrow::compute::CastOptions::Safe());
            if (!castResult.ok())
            {
                throw InputError("Failed to cast column '" + fieldName
                                 + "' to double: " + castResult.status().ToString());
            }
            column = castResult->chunked_array();
        }

        int64_t rowOffset = 0;
        for (const auto& chunk: column->chunks())
        {
            const auto& arr = std::static_pointer_cast<arrow::DoubleArray>(chunk);
            for (int64_t i = 0; i < arr->length(); ++i)
            {
                if (arr->IsNull(i))
                {
                    throw InputError("Null value at row " + std::to_string(rowOffset + i)
                                     + " in column '" + fieldName + "' of Parquet file: "
                                     + filename.string());
                }
                columns[colIdx][rowOffset + i] = arr->Value(i);
            }
            rowOffset += arr->length();
        }
    }

    return columns;
}

bool hasRightExtension(const std::filesystem::directory_entry& e)
{
    auto ext = e.path().extension();
    return (ext == ".csv") || (ext == ".tsv") || (ext == ".parquet");
}

DataSeriesRepository DataSeriesRepoImporter::importFromDirectory(const std::filesystem::path& path,
                                                                 char csvSeparator)
{
    if (!is_directory(path))
    {
        throw std::invalid_argument("Not a directory: " + path.string());
    }
    using std::views::filter;
    auto pathFilter = filter(static_cast<bool (*)(const fs::path&)>(&fs::is_regular_file));

    DataSeriesRepository repo{};
    for (auto paths = std::filesystem::directory_iterator{path};
         const auto& entry: paths | pathFilter)
    {
        if (!hasRightExtension(entry))
        {
            continue;
        }
        const auto& entryPath = entry.path();
        auto data = (entryPath.extension() == ".parquet") ? readParquet(entryPath)
                                                          : readCSV(entryPath, csvSeparator);
        auto timeSeriesSet = std::make_unique<TimeSeriesSet>(entryPath.stem().string(),
                                                             std::move(data));
        repo.addDataSeries(std::move(timeSeriesSet));
    }
    return repo;
}

} // namespace Antares::IO::Inputs::DataSeriesCsvImporter
