#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <vector>

#include "candle.hpp"
#include "register.hpp"

namespace py = pybind11;

namespace mab
{
class CandlePython : public Candle
{
   public:
	CandlePython(CANdleBaudrate_E canBaudrate, bool printVerbose = true, mab::BusType_E busType = BusType_E::USB, const std::string device = "") : Candle(canBaudrate, printVerbose, busType, device) {}
	CandlePython(CANdleBaudrate_E canBaudrate, bool printVerbose, mab::Bus* bus) : Candle(canBaudrate, printVerbose, bus) {}
	virtual ~CandlePython() = default;

	float readMd80Register_(uint16_t canId, Md80Reg_E regId, float regValue)
	{
		(void)regValue;
		md80Register->read(canId, regId, regValue);
		return regValue;
	}

	int64_t readMd80Register_(uint16_t canId, Md80Reg_E regId, int64_t regValue)
	{
		(void)regValue;
		int64_t regValue_i64 = 0;

		md80Register->read(canId, regId, regValue_i64);

		if (md80Register->getType(regId) == Register::type::I8)
			return *(uint8_t*)&regValue_i64;
		else if (md80Register->getType(regId) == Register::type::I16)
			return *(uint16_t*)&regValue_i64;
		else if (md80Register->getType(regId) == Register::type::I32)
			return *(uint32_t*)&regValue_i64;
		else
			return regValue_i64;
	}

	std::string readMd80Register_(uint16_t canId, Md80Reg_E regId, char* regValue)
	{
		(void)regValue;
		char regValue_[64] = {0};
		md80Register->read(canId, regId, regValue_);
		return std::string(regValue_);
	}

	bool writeMd80Register_(uint16_t canId, Md80Reg_E regId, float regValue)
	{
		return md80Register->write(canId, regId, regValue);
	}

	bool writeMd80Register_(uint16_t canId, Md80Reg_E regId, uint32_t regValue)
	{
		return md80Register->write(canId, regId, regValue);
	}

	bool writeMd80Register_(uint16_t canId, Md80Reg_E regId, int32_t regValue)
	{
		return md80Register->write(canId, regId, regValue);
	}

	bool writeMd80Register_(uint16_t canId, Md80Reg_E regId, std::string regValue)
	{
		char regValue_[64] = {0};
		memcpy(regValue_, regValue.c_str(), regValue.length());
		return md80Register->write(canId, regId, regValue_);
	}
};
}  // namespace mab

PYBIND11_MODULE(pyCandle, m)
{
	m.doc() = "pyCandle module for interfacing with md80 drives using Python";

	py::enum_<mab::CANdleBaudrate_E>(m, "CANdleBaudrate_E")
		.value("CAN_BAUD_1M", mab::CAN_BAUD_1M)
		.value("CAN_BAUD_2M", mab::CAN_BAUD_2M)
		.value("CAN_BAUD_5M", mab::CAN_BAUD_5M)
		.value("CAN_BAUD_8M", mab::CAN_BAUD_8M)
		.export_values();

	py::enum_<mab::Md80Mode_E>(m, "Md80Mode_E")
		.value("IDLE", mab::IDLE)
		.value("POSITION_PID", mab::POSITION_PID)
		.value("VELOCITY_PID", mab::VELOCITY_PID)
		.value("DEPRECATED", mab::DEPRECATED)
		.value("IMPEDANCE", mab::IMPEDANCE)
		.export_values();

	py::enum_<mab::CANdleMode_E>(m, "CANdleMode_E")
		.value("CONFIG", mab::CONFIG)
		.value("UPDATE", mab::UPDATE)
		.export_values();

	py::enum_<mab::BusType_E>(m, "BusType_E")
		.value("USB", mab::BusType_E::USB)
		.value("SPI", mab::BusType_E::SPI)
		.value("UART", mab::BusType_E::UART)
		.export_values();

	py::enum_<mab::Md80Reg_E>(m, "Md80Reg_E")
		.value("canId", mab::Md80Reg_E::canId)
		.value("canBaudrate", mab::Md80Reg_E::canBaudrate)
		.value("canWatchdog", mab::Md80Reg_E::canWatchdog)
		.value("canTermination", mab::Md80Reg_E::canTermination)
		.value("motorName", mab::Md80Reg_E::motorName)
		.value("motorPolePairs", mab::Md80Reg_E::motorPolePairs)
		.value("motorKt", mab::Md80Reg_E::motorKt)
		.value("motorKt_a", mab::Md80Reg_E::motorKt_a)
		.value("motorKt_b", mab::Md80Reg_E::motorKt_b)
		.value("motorKt_c", mab::Md80Reg_E::motorKt_c)
		.value("motorIMax", mab::Md80Reg_E::motorIMax)
		.value("motorGearRatio", mab::Md80Reg_E::motorGearRatio)
		.value("motorTorgueBandwidth", mab::Md80Reg_E::motorTorgueBandwidth)
		.value("motorFriction", mab::Md80Reg_E::motorFriction)
		.value("motorStiction", mab::Md80Reg_E::motorStiction)
		.value("motorResistance", mab::Md80Reg_E::motorResistance)
		.value("motorInductance", mab::Md80Reg_E::motorInductance)
		.value("motorKV", mab::Md80Reg_E::motorKV)
		.value("motorCalibrationMode", mab::Md80Reg_E::motorCalibrationMode)
		.value("motorThermistorType", mab::Md80Reg_E::motorThermistorType)

		.value("outputEncoder", mab::Md80Reg_E::outputEncoder)
		.value("outputEncoderDir", mab::Md80Reg_E::outputEncoderDir)
		.value("outputEncoderDefaultBaud", mab::Md80Reg_E::outputEncoderDefaultBaud)
		.value("outputEncoderVelocity", mab::Md80Reg_E::outputEncoderVelocity)
		.value("outputEncoderPosition", mab::Md80Reg_E::outputEncoderPosition)
		.value("outputEncoderMode", mab::Md80Reg_E::outputEncoderMode)
		.value("outputEncoderCalibrationMode", mab::Md80Reg_E::outputEncoderCalibrationMode)

		.value("motorPosPidKp", mab::Md80Reg_E::motorPosPidKp)
		.value("motorPosPidKi", mab::Md80Reg_E::motorPosPidKi)
		.value("motorPosPidKd", mab::Md80Reg_E::motorPosPidKd)
		.value("motorPosPidOutMax", mab::Md80Reg_E::motorPosPidOutMax)
		.value("motorPosPidWindup", mab::Md80Reg_E::motorPosPidWindup)

		.value("motorVelPidKp", mab::Md80Reg_E::motorVelPidKp)
		.value("motorVelPidKi", mab::Md80Reg_E::motorVelPidKi)
		.value("motorVelPidKd", mab::Md80Reg_E::motorVelPidKd)
		.value("motorVelPidOutMax", mab::Md80Reg_E::motorVelPidOutMax)
		.value("motorVelPidWindup", mab::Md80Reg_E::motorVelPidWindup)

		.value("motorImpPidKp", mab::Md80Reg_E::motorImpPidKp)
		.value("motorImpPidKd", mab::Md80Reg_E::motorImpPidKd)
		.value("motorImpPidOutMax", mab::Md80Reg_E::motorImpPidOutMax)

		.value("mainEncoderVelocity", mab::Md80Reg_E::mainEncoderVelocity)
		.value("mainEncoderPosition", mab::Md80Reg_E::mainEncoderPosition)

		.value("runSaveCmd", mab::Md80Reg_E::runSaveCmd)
		.value("runTestMainEncoderCmd", mab::Md80Reg_E::runTestMainEncoderCmd)
		.value("runTestOutputEncoderCmd", mab::Md80Reg_E::runTestOutputEncoderCmd)
		.value("runCalibrateCmd", mab::Md80Reg_E::runCalibrateCmd)
		.value("runCalibrateOutpuEncoderCmd", mab::Md80Reg_E::runCalibrateOutpuEncoderCmd)
		.value("runCalibratePiGains", mab::Md80Reg_E::runCalibratePiGains)

		.value("calOutputEncoderStdDev", mab::Md80Reg_E::calOutputEncoderStdDev)
		.value("calOutputEncoderMinE", mab::Md80Reg_E::calOutputEncoderMinE)
		.value("calOutputEncoderMaxE", mab::Md80Reg_E::calOutputEncoderMaxE)
		.value("calMainEncoderStdDev", mab::Md80Reg_E::calMainEncoderStdDev)
		.value("calMainEncoderMinE", mab::Md80Reg_E::calMainEncoderMinE)
		.value("calMainEncoderMaxE", mab::Md80Reg_E::calMainEncoderMaxE)

		.value("shuntResistance", mab::Md80Reg_E::shuntResistance)

		.value("buildDate", mab::Md80Reg_E::buildDate)
		.value("commitHash", mab::Md80Reg_E::commitHash)
		.value("firmwareVersion", mab::Md80Reg_E::firmwareVersion)
		.value("hardwareVersion", mab::Md80Reg_E::hardwareVersion)
		.value("bridgeType", mab::Md80Reg_E::bridgeType)
		.value("errorVector", mab::Md80Reg_E::errorVector)
		.value("mosfetTemperature", mab::Md80Reg_E::mosfetTemperature)
		.value("motorTemperature ", mab::Md80Reg_E::motorTemperature)
		.value("mainEncoderErrors", mab::Md80Reg_E::mainEncoderErrors)
		.value("outputEncoderErrors", mab::Md80Reg_E::outputEncoderErrors)
		.value("calibrationErrors", mab::Md80Reg_E::calibrationErrors)
		.value("bridgeErrors", mab::Md80Reg_E::bridgeErrors)
		.value("hardwareErrors", mab::Md80Reg_E::hardwareErrors)
		.value("communicationErrors ", mab::Md80Reg_E::communicationErrors)
		.export_values();

	py::class_<mab::ImpedanceControllerGains_t>(m, "ImpedanceControllerGains_t")
		.def(py::init())
		.def_readwrite("kp", &mab::ImpedanceControllerGains_t::kp)
		.def_readwrite("kd", &mab::ImpedanceControllerGains_t::kd)
		.def_readwrite("outMax", &mab::ImpedanceControllerGains_t::outMax);

	py::class_<mab::PidControllerGains_t>(m, "PidControllerGains_t")
		.def(py::init())
		.def_readwrite("kp", &mab::PidControllerGains_t::kp)
		.def_readwrite("ki", &mab::PidControllerGains_t::ki)
		.def_readwrite("kd", &mab::PidControllerGains_t::kd)
		.def_readwrite("intWindup", &mab::PidControllerGains_t::intWindup)
		.def_readwrite("outMax", &mab::PidControllerGains_t::outMax);

	py::class_<mab::regRO_st>(m, "regRO_st")
		.def(py::init())
		.def_readwrite("firmwareVersion", &mab::regRO_st::firmwareVersion)
		.def_readwrite("hardwareVersion", &mab::regRO_st::hardwareVersion)
		.def_readwrite("buildDate", &mab::regRO_st::buildDate)
		// .def_readwrite("commitHash", &mab::regRO_st::commitHash) //char array doesnt work, see how char arrays registers are handled in the example12
		.def_readwrite("bridgeType", &mab::regRO_st::bridgeType)
		.def_readwrite("resistance", &mab::regRO_st::resistance)
		.def_readwrite("inductance", &mab::regRO_st::inductance)
		.def_readwrite("errorVector", &mab::regRO_st::errorVector)
		.def_readwrite("mosfetTemperature", &mab::regRO_st::mosfetTemperature)
		.def_readwrite("motorTemperature", &mab::regRO_st::motorTemperature)
		.def_readwrite("mainEncoderVelocity", &mab::regRO_st::mainEncoderVelocity)
		.def_readwrite("mainEncoderPosition", &mab::regRO_st::mainEncoderPosition)
		.def_readwrite("outputEncoderVelocity", &mab::regRO_st::outputEncoderVelocity)
		.def_readwrite("outputEncoderPosition", &mab::regRO_st::outputEncoderPosition)
		.def_readwrite("calOutputEncoderStdDev", &mab::regRO_st::calOutputEncoderStdDev)
		.def_readwrite("calOutputEncoderMinE", &mab::regRO_st::calOutputEncoderMinE)
		.def_readwrite("calOutputEncoderMaxE", &mab::regRO_st::calOutputEncoderMaxE)
		.def_readwrite("calMainEncoderStdDev", &mab::regRO_st::calMainEncoderStdDev)
		.def_readwrite("calMainEncoderMinE", &mab::regRO_st::calMainEncoderMinE)
		.def_readwrite("calMainEncoderMaxE", &mab::regRO_st::calMainEncoderMaxE)
		.def_readwrite("mainEncoderErrors", &mab::regRO_st::mainEncoderErrors)
		.def_readwrite("outputEncoderErrors", &mab::regRO_st::outputEncoderErrors)
		.def_readwrite("calibrationErrors", &mab::regRO_st::calibrationErrors)
		.def_readwrite("bridgeErrors", &mab::regRO_st::bridgeErrors)
		.def_readwrite("hardwareErrors", &mab::regRO_st::hardwareErrors)
		.def_readwrite("communicationErrors", &mab::regRO_st::communicationErrors)
		.def_readwrite("shuntResistance", &mab::regRO_st::shuntResistance);

	py::class_<mab::regRW_st>(m, "regRW_st")
		.def(py::init())
		.def_readwrite("canId", &mab::regRW_st::canId)
		.def_readwrite("canBaudrate", &mab::regRW_st::canBaudrate)
		.def_readwrite("canWatchdog", &mab::regRW_st::canWatchdog)
		.def_readwrite("canTermination", &mab::regRW_st::canTermination)
		.def_readwrite("polePairs", &mab::regRW_st::polePairs)
		.def_readwrite("motorKV", &mab::regRW_st::motorKV)
		.def_readwrite("motorCalibrationMode", &mab::regRW_st::motorCalibrationMode)
		.def_readwrite("motorThermistorType", &mab::regRW_st::motorThermistorType)
		.def_readwrite("motorKt", &mab::regRW_st::motorKt)
		.def_readwrite("motorKt_a", &mab::regRW_st::motorKt_a)
		.def_readwrite("motorKt_b", &mab::regRW_st::motorKt_b)
		.def_readwrite("motorKt_c", &mab::regRW_st::motorKt_c)
		.def_readwrite("iMax", &mab::regRW_st::iMax)
		.def_readwrite("gearRatio", &mab::regRW_st::gearRatio)
		.def_readwrite("outputEncoder", &mab::regRW_st::outputEncoder)
		.def_readwrite("outputEncoderMode", &mab::regRW_st::outputEncoderMode)
		.def_readwrite("outputEncoderCalibrationMode", &mab::regRW_st::outputEncoderCalibrationMode)
		.def_readwrite("outputEncoderDir", &mab::regRW_st::outputEncoderDir)
		.def_readwrite("torqueBandwidth", &mab::regRW_st::torqueBandwidth)
		.def_readwrite("outputEncoderDefaultBaud", &mab::regRW_st::outputEncoderDefaultBaud)
		.def_readwrite("friction", &mab::regRW_st::friction)
		.def_readwrite("stiction", &mab::regRW_st::stiction)
		.def_readwrite("motorShutdownTemp", &mab::regRW_st::motorShutdownTemp)
		.def_readwrite("impedancePdGains", &mab::regRW_st::impedancePdGains)
		.def_readwrite("velocityPidGains", &mab::regRW_st::velocityPidGains)
		.def_readwrite("positionPidGains", &mab::regRW_st::positionPidGains)
		.def_readwrite("runSaveCmd", &mab::regRW_st::runSaveCmd)
		.def_readwrite("runTestOutputEncoderCmd", &mab::regRW_st::runTestOutputEncoderCmd)
		.def_readwrite("runTestMainEncoderCmd", &mab::regRW_st::runTestMainEncoderCmd)
		.def_readwrite("runCalibrateCmd", &mab::regRW_st::runCalibrateCmd)
		.def_readwrite("runCalibrateOutpuEncoderCmd", &mab::regRW_st::runCalibrateOutpuEncoderCmd)
		.def_readwrite("runCalibratePiGains", &mab::regRW_st::runCalibratePiGains);

	py::class_<mab::regWrite_st>(m, "regWrite_st")
		.def(py::init())
		.def_readwrite("RW", &mab::regWrite_st::RW);

	py::class_<mab::regRead_st>(m, "regRead_st")
		.def(py::init())
		.def_readwrite("RO", &mab::regRead_st::RO)
		.def_readwrite("RW", &mab::regRead_st::RW);

	py::class_<mab::Md80>(m, "Md80")
		.def("setPositionControllerParams", &mab::Md80::setPositionControllerParams)
		.def("setVelocityControllerParams", &mab::Md80::setVelocityControllerParams)
		.def("setImpedanceControllerParams", &mab::Md80::setImpedanceControllerParams)
		.def("setMaxTorque", &mab::Md80::setMaxTorque)
		.def("setMaxVelocity", &mab::Md80::setMaxVelocity)
		.def("setTargetPosition", &mab::Md80::setTargetPosition)
		.def("setTargetVelocity", &mab::Md80::setTargetVelocity)
		.def("setTorque", &mab::Md80::setTorque)
		.def("getErrorVector", &mab::Md80::getErrorVector)
		.def("getId", &mab::Md80::getId)
		.def("getPosition", &mab::Md80::getPosition)
		.def("getVelocity", &mab::Md80::getVelocity)
		.def("getOutputEncoderPosition", &mab::Md80::getOutputEncoderPosition)
		.def("getOutputEncoderVelocity", &mab::Md80::getOutputEncoderVelocity)
		.def("getTorque", &mab::Md80::getTorque)
		.def("getTemperature", &mab::Md80::getTemperature)
		.def("getReadReg", &mab::Md80::getReadReg)
		.def("getWriteReg", &mab::Md80::getWriteReg);

	py::class_<mab::CandlePython>(m, "Candle")
		.def(py::init<mab::CANdleBaudrate_E, bool>())
		.def(py::init<mab::CANdleBaudrate_E, bool, mab::BusType_E>())
		.def("getVersion", &mab::CandlePython::getVersion)
		.def("getDeviceId", &mab::CandlePython::getDeviceId)
		.def_readwrite("md80s", &mab::CandlePython::md80s)
		.def("setVebose", &mab::CandlePython::setVebose)
		.def("getActualCommunicationFrequency", &mab::CandlePython::getActualCommunicationFrequency)
		.def("setTransmitDelayUs", &mab::CandlePython::setTransmitDelayUs)
		.def("ping", py::overload_cast<>(&mab::CandlePython::ping))
		.def("ping", py::overload_cast<mab::CANdleBaudrate_E>(&mab::CandlePython::ping))
		.def("sendGenericFDCanFrame", &mab::CandlePython::sendGenericFDCanFrame)
		.def("addMd80", &mab::CandlePython::addMd80, py::arg("canID"), py::arg("printFailure") = false)
		.def("configCandleBaudrate", &mab::CandlePython::configCandleBaudrate)
		.def("configMd80Can", &mab::CandlePython::configMd80Can)
		.def("configMd80SetCurrentLimit", &mab::Candle::configMd80SetCurrentLimit)
		.def("configMd80Save", &mab::CandlePython::configMd80Save)
		.def("configMd80Blink", &mab::CandlePython::configMd80Blink)
		.def("configMd80TorqueBandwidth", &mab::CandlePython::configMd80TorqueBandwidth)
		.def("controlMd80SetEncoderZero", py::overload_cast<mab::Md80&>(&mab::CandlePython::controlMd80SetEncoderZero))
		.def("controlMd80SetEncoderZero", py::overload_cast<uint16_t>(&mab::CandlePython::controlMd80SetEncoderZero))
		.def("controlMd80Mode", py::overload_cast<mab::Md80&, mab::Md80Mode_E>(&mab::CandlePython::controlMd80Mode))
		.def("controlMd80Mode", py::overload_cast<uint16_t, mab::Md80Mode_E>(&mab::CandlePython::controlMd80Mode))
		.def("controlMd80Enable", py::overload_cast<mab::Md80&, bool>(&mab::CandlePython::controlMd80Enable))
		.def("controlMd80Enable", py::overload_cast<uint16_t, bool>(&mab::CandlePython::controlMd80Enable))
		.def("getMd80FromList", &mab::Candle::getMd80FromList)
		.def("begin", &mab::CandlePython::begin)
		.def("end", &mab::CandlePython::end)
		.def("reset", &mab::CandlePython::reset)
		.def("setupMd80Calibration", &mab::CandlePython::setupMd80Calibration)
		.def("setupMd80Diagnostic", &mab::CandlePython::setupMd80Diagnostic)
		.def("getCurrentBaudrate", &mab::CandlePython::getCurrentBaudrate)
		.def("checkMd80ForBaudrate", &mab::CandlePython::checkMd80ForBaudrate)
		.def("writeMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, std::string>(&mab::CandlePython::writeMd80Register_))
		.def("writeMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, float>(&mab::CandlePython::writeMd80Register_))
		.def("writeMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, uint32_t>(&mab::CandlePython::writeMd80Register_))
		.def("writeMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, int32_t>(&mab::CandlePython::writeMd80Register_))
		.def("readMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, char*>(&mab::CandlePython::readMd80Register_))
		.def("readMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, float>(&mab::CandlePython::readMd80Register_))
		.def("readMd80Register", py::overload_cast<uint16_t, mab::Md80Reg_E, int64_t>(&mab::CandlePython::readMd80Register_));
}
