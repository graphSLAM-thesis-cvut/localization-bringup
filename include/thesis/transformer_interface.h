#ifndef TRANSFORMER_INTERFACE_H
#define TRANSFORMER_INTERFACE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class TransformerInterface{
  private:
	tf::TransformListener listener;
	std::string fromFrame;
	std::string toFrame;
	
  public:
  	TransformerInterface( std::string _fromFrame,	std::string _toFrame ){
		fromFrame = _fromFrame;
		toFrame = _toFrame;
	};
  
	bool getTransformFromTF2( tf::StampedTransform* outtransform ){
		bool succesfullyGotTrans = true;
		try{
			listener.lookupTransform( fromFrame, toFrame, ros::Time(0), *outtransform);
		}catch(tf::TransformException ex){
			ros::Duration(0.1).sleep();
			succesfullyGotTrans = false;
			fprintf(stdout,"TF transform (from %s to %s) not found!\n", fromFrame.c_str(), toFrame.c_str() );
	   	}
	  	return succesfullyGotTrans;
	};
	
};


#endif
