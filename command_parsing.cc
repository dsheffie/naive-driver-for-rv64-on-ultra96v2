#include <boost/program_options.hpp>
#include <string>
#include <cstdint>
#include <iostream>

bool cmdline(int argc,
	     char *argv[],
	     bool &initialize,
	     bool &silent,
	     std::string &chpt_name,
	     uint32_t &max_fetches,
	     uint64_t &max_iters,
	     bool &sgi_mode,
	     bool &single_step,
	     std::string &arcs_image,
	     std::string &start_pc,
	     std::string &cimg_name,
	     std::string &disk_name,
	     std::string &rom_image,
	     std::string &rom_pa,
	     std::string &newport_display,
	     int &newport_cpu) {
  
  namespace po = boost::program_options;
  po::options_description desc("Options");  
  desc.add_options() 
    ("help,h", "Print help messages") 
    ("initialize,i", po::value<bool>(&initialize)->default_value(true), "initialize")
    ("silent", po::value<bool>(&silent)->default_value(false), "be quiet")
    ("file,f", po::value<std::string>(&chpt_name), "checkpoint filename")
    ("fetches", po::value<uint32_t>(&max_fetches)->default_value(0), "max fetches")
    ("maxiters", po::value<uint64_t>(&max_iters)->default_value(~0UL), "max sample loops")
    ("sgi", po::value<bool>(&sgi_mode)->default_value(false), "sgi memory map")
    ("step,s", po::value<bool>(&single_step)->default_value(false), "single-step")
    ("arcs", po::value<std::string>(&arcs_image)->default_value(""), "arcs firmware image to load at phys 0x1000 (empty = none; arcs_fw.bin for Linux, arcs_irix.bin for IRIX)")
    ("start-pc", po::value<std::string>(&start_pc)->default_value(""), "override start PC, e.g. 0xa0003000 (empty = ELF entry / sgi default)")
    ("cimg", po::value<std::string>(&cimg_name)->default_value(""), "checkpoint memory image (ckpt2preamble .cimg); skips ELF, forces sgi + pc=0xbfc00000, load preamble via --arcs")
    ("disk", po::value<std::string>(&disk_name)->default_value("/home/root/irix65-clean.img"), "disk image")
      ("rom", po::value<std::string>(&rom_image)->default_value(""), "filesystem image preloaded into DRAM for the hcart block device (no DMA); empty = none")
      ("rom-pa", po::value<std::string>(&rom_pa)->default_value("0x0c000000"), "MIPS physical address for --rom; must sit above the RAM the kernel is told about")
    ("newport-display", po::value<std::string>(&newport_display)->default_value(""), "X display for the Newport window, e.g. :2 (empty = headless)")
    ("newport-cpu", po::value<int>(&newport_cpu)->default_value(3), "A53 core for the GIO-window service thread (-1 = unpinned)")
    ;
  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm); 
    if(vm.count("help")) {
      /* without this, --help fell through and RAN: zeroed DRAM and reset a live core */
      std::cout << desc << "\n";
      return false;
    }
  }
  catch(po::error &e) {
    std::cerr << "command-line error : " << e.what() << "\n";
    return false;
  }
  return true;
}
