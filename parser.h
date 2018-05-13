#ifndef PARSER_H
#define PARSER_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>

class Parser
{

 private:

 public:

  Parser();
  ~Parser();
  
  void parse(char data[],float &t, float &Rollset, float &roll_p, float &roll_i, float &roll_d, float &k1, float &k2, float &k3);

};

extern Parser parser;

#endif
