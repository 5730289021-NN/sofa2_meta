

class Motor
{
public:
	Motor(int id, int cw_lim, int ccw_lim, int inv, int scale): id(id), cw_lim(cw_lim), ccw_lim(ccw_lim), inv(inv), scale(scale), cur_power(0), scaled_power(0){
		cur_pos = -id;
	}
	std::string getInfo(){
		std::stringstream ss;
		ss << "Motor ID:" << id << " CW_LIM:" << cw_lim << " CCW_LIM:" << ccw_lim << " INV:" << inv;
		return ss.str();
	}
	void updatePosition(int pos){
		cur_pos = pos;
	}
	int getID() const{
		return id;
	}
	std::string getStringID() const{
		return std::to_string(id);
	}
	int getPosition() const{
		return cur_pos;
	}
	void setPower(double power) {
		cur_power = inv * power;
		if(cur_power > 1) cur_power = 1;
		else if(cur_power < -1) cur_power = -1;
		/*Check constraint*/
		if(cur_pos < cw_lim && cur_power < 0 || cur_pos > ccw_lim && cur_power > 0) {
			//When trying to go out of bound
			cur_power = 0;
			ROS_WARN_STREAM("Motor " << id << " : " << cur_pos << " not within (" << cw_lim << ", " << ccw_lim << ")");
		}

		scaled_power = std::abs(cur_power) * scale;
		//ROS_DEBUG_STREAM("Scaled Power: " << scaled_power << " for ID: " << id);
	}
	
	uint8_t getDriveByteLow() const {
		//ROS_DEBUG_STREAM("Motor " << getID() << " , Return Byte Low: " << int(scaled_power % 256) << " from Scaled Power : " << scaled_power << " Scale : " << scale << " Power : " << cur_power);
		return scaled_power % 256;
	}

	uint8_t getDriveByteHigh() const {
		uint8_t hb = scaled_power / 256;
		if(cur_power * inv < 0) {
			hb += 4;
		}
		//ROS_DEBUG_STREAM("Motor " << getID() << " , Return Byte High: " << int(hb) << " from Scaled Power : " << scaled_power);
		return hb;
	}

	void printMotorRecord(bool printInfo=false) const{
		if(printInfo)
			ROS_INFO_STREAM("Record ID: " << id << " Position: " << cur_pos <<" Address: "<< this);
		else
			ROS_DEBUG_STREAM("Record ID: " << id << " Position: " << cur_pos <<" Address: "<< this);
	}

private:
	int id;
	int cw_lim;
	int ccw_lim;
	int inv;
	int cur_pos;
	int scale;
	double cur_power;
	int scaled_power;
};