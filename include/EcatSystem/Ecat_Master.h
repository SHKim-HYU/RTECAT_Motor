/*
 * Ecat_Master.h
 *
 *  Created on: 2024. 03. 04.
 *      Author: Sunhong Kim
 */

#ifndef ECATSYSTEM_ECAT_MASTER_H_
#define ECATSYSTEM_ECAT_MASTER_H_

#include "ecrt.h"
#include <string>
#include <vector>
#include <map>
#include <stdio.h>
#include <stdint.h>

#include "CiA402_Object_Dictionary.h"
#include "Ecat_iServo.h"
#include "PropertyDefinition.h"

class Master {
public:
	Master(const int master = 0);
	virtual ~Master();
	void SDOread(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data);
	void SDOwrite(uint16_t position, uint16_t index, uint8_t subindex, uint8_t *data);
	void addSlave(uint16_t alias, uint16_t position, Slave* slave);
	void addSlaveiServo(uint16_t alias, uint16_t position, Ecat_iServo* slave);
	void activate();
	void activateWithDC(uint8_t RefPosition, uint32_t SyncCycleNano);
	void SyncEcatMaster(uint64_t RefTime);
	void deactivate();
	void update(unsigned int domain = 0);
	void TxUpdate(unsigned int domain = 0);
	void RxUpdate(unsigned int domain = 0);

	void checkDomainState(unsigned int domain = 0);
	void checkMasterState();
	void checkSlaveStates();


	// SDO_Functions
	// read
	int SDOread_ENCODER_RESOLUTION(int position);
	int SDOread_RATE_CURRENT(int position);
	int SDOread_TORQUE_CONSTANT(int position);
	int SDOread_MOTOR_DIRECTION(int position);
	// write
	void SDOwrite_MODE_OF_OPERATION(int position, int mode);
	void SDOwrite_HOMING_OFFSET(int position, int data);
	void SDOwrite_HOMING_POSITION(int position, int data);
	void SDOwrite_HOMING_NEG_CURRENT_LIMIT(int position, int data);
	void SDOwrite_HOMING_POS_CURRENT_LIMIT(int position, int data);
	void SDOwrite_HOMING_METHOD(int position, int data);
	void SDOwrite_HOMING_SPEED(int position, int data);
	void SDOwrite_HOMING_ACCELERATION(int position, int data);


private:
	volatile bool isRunning = false;

	struct DomainInfo;
	void registerPDOInDomain(uint16_t alias, uint16_t position, std::vector<unsigned int>& channel_indices,	DomainInfo* domain_info, Slave* slave);


	static void printWarning(const std::string& message);
	static void printWarning(const std::string& message, int position);

	ec_master_t *p_master;
	ec_master_state_t m_master_state = {};

	struct DomainInfo{
		DomainInfo(ec_master_t* master);
		~DomainInfo();

		ec_domain_t *domain = NULL;
		ec_domain_state_t domain_state = {};
		uint8_t *domain_pd = NULL;

		std::vector<ec_pdo_entry_reg_t> domain_regs;

		struct Entry{
			Slave* slave = NULL;
			int num_pdos = 0;
			unsigned int* offset = NULL;
			unsigned int* bit_position = NULL;
		};

		std::vector<Entry> entries;
	};

	std::map<unsigned int, DomainInfo*> m_domain_info;

	struct SlaveInfo{
		Slave* slave = NULL;
		ec_slave_config_t* config = NULL;
		ec_slave_config_state_t config_state = {0};
	};

	std::vector<SlaveInfo> m_slave_info;

    /** counter of control loops */
    unsigned long long update_counter_ = 0;

    /** frequency to check for master or slave state change.
     *  state checked every frequency_ control loops */
    unsigned int check_state_frequency_ = 100;


};

#endif /* ECATSYSTEM_ECAT_MASTER_H_ */
