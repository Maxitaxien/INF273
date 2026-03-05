#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

// Store CSV row per algorithm
struct CsvResult
{
    std::vector<std::string> headers;
    std::vector<std::string> values;
};

// Store TXT solution string
struct TxtResult
{
    std::string algorithm;
    std::string content;
};

struct DatasetKey
{
    char type; // 'F' or 'R'
    int size;  // 10, 20, 50, 100
};

// Parse "F_10" or "r100" etc.
DatasetKey parseDataset(const std::string &s)
{
    DatasetKey k;
    k.type = std::tolower(s[0]);

    // Extract number
    std::string num;
    for (char c : s)
    {
        if (std::isdigit(c))
            num += c;
    }
    k.size = std::stoi(num);
    return k;
}

// dataset -> algorithm -> csv result
using DatasetCsvMap = std::map<std::string, std::map<std::string, CsvResult>>;

// dataset -> list of txt results
using DatasetTxtMap = std::map<std::string, std::vector<TxtResult>>;

// Split CSV line
std::vector<std::string> splitCSV(const std::string &line)
{
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        result.push_back(item);
    }
    return result;
}
bool datasetLess(const std::string &a, const std::string &b)
{
    auto A = parseDataset(a);
    auto B = parseDataset(b);

    // F before R
    if (A.type != B.type)
        return A.type < B.type;

    // Then numeric order
    return A.size < B.size;
}

// Read CSV file and extract header + row
CsvResult parseCSV(const fs::path &file)
{
    std::ifstream in(file);
    CsvResult res;
    std::string line;

    std::getline(in, line); // skip first line

    std::getline(in, line);
    res.headers = splitCSV(line);

    std::getline(in, line);
    res.values = splitCSV(line);

    // Remove empty first column if present
    if (!res.headers.empty() && res.headers[0].find("Average") == std::string::npos)
    {
        res.headers.erase(res.headers.begin());
        res.values.erase(res.values.begin());
    }

    return res;
}

// Read entire txt file
std::string readTXT(const fs::path &file)
{
    std::ifstream in(file);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

void create_markdown_tables(
    std::string rootDir)
{
    fs::path root(rootDir);

    DatasetCsvMap csvData;
    DatasetTxtMap txtData;

    // Iterate algorithms
    for (auto &algEntry : fs::directory_iterator(root))
    {
        if (!algEntry.is_directory())
            continue;

        std::string algorithm = algEntry.path().filename().string();

        for (auto &fileEntry : fs::directory_iterator(algEntry))
        {
            auto path = fileEntry.path();
            std::string dataset = path.stem().string(); // e.g., F_10

            if (path.extension() == ".csv")
            {
                csvData[dataset][algorithm] = parseCSV(path);
            }
            else if (path.extension() == ".txt")
            {
                TxtResult tr;
                tr.algorithm = algorithm;
                tr.content = readTXT(path);
                txtData[dataset].push_back(tr);
            }
        }
    }

    // Create markdown file
    fs::path mdFile = root / "results.md";
    std::ofstream out(mdFile);

    // Sort datasets properly
    std::vector<std::string> datasets;
    for (auto &[dataset, _] : csvData)
        datasets.push_back(dataset);

    std::sort(datasets.begin(), datasets.end(), datasetLess);

    for (auto &dataset : datasets)
    {
        auto &algMap = csvData[dataset];

        out << "\n";
        out << "## Dataset: " << dataset << "\n\n";

        auto first = algMap.begin()->second.headers;

        out << "| Algorithm |";
        for (auto &h : first)
            out << " " << h << " |";
        out << "\n|-----------|";
        for (size_t i = 0; i < first.size(); i++)
            out << "-----------|";
        out << "\n";

        for (auto &[alg, res] : algMap)
        {
            out << "| " << alg << " |";
            for (auto &v : res.values)
                out << " " << v << " |";
            out << "\n";
        }

        out << "\n### Solutions\n\n";
        for (auto &txt : txtData[dataset])
        {
            out << "**" << txt.algorithm << "**\n";
            out << "```\n"
                << txt.content << "\n```\n\n";
        }
    }

    std::cout << "Markdown written to " << mdFile << "\n";
}
