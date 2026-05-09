#include <boost/program_options.hpp>
#include <string>
#include <cstdint>
#include <iostream>

bool cmdline(int argc,
	     char *argv[],
	     bool &initialize,
	     std::string &chpt_name,
	     uint32_t &max_fetches) {
  
  namespace po = boost::program_options;
  po::options_description desc("Options");  
  desc.add_options() 
    ("help,h", "Print help messages") 
    ("initialize,i", po::value<bool>(&initialize)->default_value(true), "initialize") 
    ("file,f", po::value<std::string>(&chpt_name), "checkpoint filename")
    ("fetches", po::value<uint32_t>(&max_fetches)->default_value(0), "max fetches")
    ;  
  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm); 
  }
  catch(po::error &e) {
    std::cerr << "command-line error : " << e.what() << "\n";
    return false;
  }
  return true;
}
