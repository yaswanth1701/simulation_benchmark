#include "log.hh"

using namespace gz; 
using namespace benchmark;


Log::Log(std::string filePath)
{
  auto options = mcap::McapWriterOptions("");
  const auto s = this->writer.open(result_name.c_str(), options);
}

