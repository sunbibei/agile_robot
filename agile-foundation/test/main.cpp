/*
 * main.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#include <string>

#include "foundation/utf.h"
#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"

#define TEST_MAIN
#ifdef TEST_MAIN

void print(const std::string& p, const std::string& l) {
  std::cout << "In Callback, " << Label::make_label(p, l) << std::endl;// ": ";
  // std::string val;
  // CfgReader::instance()->get_value(Label::make_label(p, l), "auto_inst", val);
  // std::cout << val << std::endl;

}

#include <sys/times.h>

void __auto_inst(const std::string& __p, const std::string& __type) {
  LOG_INFO << "Create instance(" << __type << " " << __p;
/*  if (!AutoInstor::instance()->make_instance(__p, __type)) {
    LOG_WARNING << "Create instance(" << __type << " " << __p << ") fail!";
  }*/
}

int main1() {
  // Just for test
  // auto auto_inst = AutoInstor::create_instance("/home/bibei/Workspaces/qr_ws/devel/lib/libqr_control_repository.so");
/*  if (nullptr == auto_inst) {
    LOG_FATAL << "Create the singleton 'AutoInstor' has failed.";
  }*/
  CfgReader::add_path("/home/bibei/Workspaces/agile_ws/src/agile_robot/agile-apps/config");
  auto cfg = CfgReader::create_instance();
  if (!cfg) {
    LOG_FATAL << "The CfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before GaitManager::init()";
  }

  cfg->add_config("pd_robot.cfg");

  auto f = [](const std::string& attr, const std::string& val) {
    printf("key: %s -> value: %s\n", attr.c_str(), val.c_str());
  };
  cfg->foreachAttr("pd.res.jnts.lft", f);

  cfg->foreachTag("pd.core.nodes.leg_node", [](const std::string& _p){
    printf("TAG: %s\n", _p.c_str());
  });
  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
//  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
//  cfg->regAttrCb("auto_inst", __auto_inst);
//  // Just for debug
//  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
//  Label::printfEveryInstance();

  return 0;
/*
  std::vector<std::string> ps = {"qr.second.third.forth.fifth.sixth.seventh.eighth.night.tenth",
      "qr.second.third.forth.fifth.sixth.scale", "qr.second.third.forth.fifth.sixth.scale2"};
  std::vector<std::string> attrs = {"attr", "abcd", "efd"};


  std::string val_str;
  double val_d;
  int val_i;
  bool val_b;
  char val_c;
  unsigned char val_uc;

  std::vector<std::string> vec_str;
  std::vector<int> vec_i;
  std::vector<double> vec_d;
  std::vector<bool> vec_b;
  std::vector<unsigned char> vec_c1;

  if (cfg->get_value("qr.fl.touchdown", "test1", vec_c1)) {
    for (const auto& val : vec_c1)
      printf("0x%02X ", val);
    printf("\n");
  } else
    std::cout << "NO FOUND" << std::endl;

  if (cfg->get_value("qr.fl.touchdown", "msg_id", val_uc)) {
      printf("0x%02X\n", val_uc);
  } else
    std::cout << "NO FOUND" << std::endl;

  return 0;

  if (cfg->get_value("qr.fl.touchdown", "auto_inst", val_str))
    std::cout << val_str << std::endl;
  else
    std::cout << "NO FOUND" << std::endl;

  if (cfg->get_value("qr.fl.touchdown", "scale", val_d))
    std::cout << val_d << std::endl;
  else
    std::cout << "NO FOUND" << std::endl;

  if (cfg->get_value("qr.fl.touchdown", "offset1", val_i, 1000))
    std::cout << val_i << std::endl;
  else
    std::cout << "NO FOUND" << std::endl;

  if (cfg->get_value("qr.fl.touchdown", "test", val_b, true))
    std::cout << val_b << std::endl;
  else
    std::cout << "NO FOUND" << std::endl;

  if (cfg->get_value("qr.fl.touchdown", "msg_id", val_c))
    printf("0x%02X\n", val_c);
  else
    std::cout << "NO FOUND" << std::endl;
  // return 0;


  cfg->get_value("qr.fl.touchdown", "auto_inst", vec_str);
  for (const auto& str : vec_str)
    std::cout << str << " ";
  std::cout << std::endl;

  int i = 0;
  std::cout << ps[i] << "." << attrs[i] << ": \n";
  std::cout << "++++++++++++++++++++++" << std::endl;
  std::cout << "++++++++++++++++++++++" << std::endl;
  cfg->get_value(ps[i], attrs[i], vec_i);
  std::cout << "++++++++++++++++++++++" << std::endl;
  for (const auto& str : vec_i)
    std::cout << str << " ";
  std::cout << std::endl;


  ++i;
  std::cout << ps[i] << "." << attrs[i] << ": \n";
  std::cout << "++++++++++++++++++++++" << std::endl;
  std::cout << "++++++++++++++++++++++" << std::endl;
  cfg->get_value(ps[i], attrs[i], vec_d);
  std::cout << "++++++++++++++++++++++" << std::endl;
  for (const auto& str : vec_d)
    std::cout << str << " ";
  std::cout << std::endl;

  ++i;
  std::cout << ps[i] << "." << attrs[i] << ": \n";
  std::cout << "++++++++++++++++++++++" << std::endl;
  std::cout << "++++++++++++++++++++++" << std::endl;
  cfg->get_value_fatal(ps[i], attrs[i], vec_b);
  std::cout << "++++++++++++++++++++++" << std::endl;
  for (const auto& str : vec_b)
    std::cout << str << " ";
  std::cout << std::endl;


  CfgReader::destroy_instance();
  return 0;*/

}

#endif
