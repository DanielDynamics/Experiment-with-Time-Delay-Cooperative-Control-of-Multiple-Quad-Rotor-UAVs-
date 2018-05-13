/*
  author: Zhentao Xie
  ------------------------------------

  take the char data as an input and
  ouputs floating points values

  * Modified from original parser created by vincennt JAUNET (vincent.jaunet@hotmail.fr)
*/

#include "parser.h"

Parser parser;

//constructor
Parser::Parser()
{
}

//destructor
Parser::~Parser()
{
}

void Parser::parse(char data[],float &t, float &Rollset, float &roll_p, float &roll_i, float &roll_d, float &k1, float &k2, float &k3)
{
	//Processing packet
  std::string packet( reinterpret_cast< char const* > (data));
  std::istringstream ss(packet);
  printf("Data in parse is : %s \n", packet.c_str());

  //Getting target values from packet
  do
    {
      std::string sub;
      ss >> sub;
      float cmd;

      if (sub == "thr:" )
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    //t = (cmd*10 + 1080);
	    t = cmd;
	    printf("Throttle is : %f\n",t);
      }
      else if(sub == "Rollset:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    Rollset = cmd;
	    printf("Rollset is changed as: %f\n",Rollset);
      }
      else if(sub == "roll_p:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    roll_p = cmd;
	    printf("roll_p is changed as: %f\n",roll_p);
      }
      else if(sub == "roll_i:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    roll_i = cmd;
	    printf("roll_i is changed as: %f\n",roll_i);
      }
      else if(sub == "roll_d:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    roll_d = cmd;
	    printf("roll_d is changed as: %f\n",roll_d);
      }
      else if(sub == "k1:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    k1 = cmd;
	    printf("k1 is changed as: %f\n",k1);
      }
      else if(sub == "k2:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    k2 = cmd;
	    printf("k2 is changed as: %f\n",k2);
      }
      else if(sub == "k3:")
      {
	    ss >> sub;
	    std::istringstream( sub ) >> cmd;
	    k3 = cmd;
	    printf("k3 is changed as: %f\n",k3);
      }
      
    } while (ss);
}

