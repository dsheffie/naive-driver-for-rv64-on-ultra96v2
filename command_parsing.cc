#include <boost/program_options.hpp>
#include <string>
#include <cstdint>
#include <iostream>

bool cmdline(int argc,
	     char *argv[],
	     bool &initialize,
	     std::string &chpt_name,
	     uint32_t &max_fetches,
	     uint64_t &max_iters,
	     bool &sgi_mode,
	     bool &single_step,
	     std::string &arcs_image,
	     std::string &start_pc) {
  
  namespace po = boost::program_options;
  po::options_description desc("Options");  
  desc.add_options() 
    ("help,h", "Print help messages") 
    ("initialize,i", po::value<bool>(&initialize)->default_value(true), "initialize") 
    ("file,f", po::value<std::string>(&chpt_name), "checkpoint filename")
    ("fetches", po::value<uint32_t>(&max_fetches)->default_value(0), "max fetches")
    ("maxiters", po::value<uint64_t>(&max_iters)->default_value(~0UL), "max sample loops")
    ("sgi", po::value<bool>(&sgi_mode)->default_value(false), "sgi memory map")
    ("step,s", po::value<bool>(&single_step)->default_value(false), "single-step")
    ("arcs", po::value<std::string>(&arcs_image)->default_value(""), "arcs firmware image to load at phys 0x1000 (empty = none; arcs_fw.bin for Linux, arcs_irix.bin for IRIX)")
    ("start-pc", po::value<std::string>(&start_pc)->default_value(""), "override start PC, e.g. 0xa0003000 (empty = ELF entry / sgi default)")
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
