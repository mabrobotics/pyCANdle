#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <vector>

#include "candle.hpp"

namespace py = pybind11;

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

	// py::enum_<mab::Md80Reg_E>(m, "Md80Reg_E")
	// 	.value("canId", mab::Md80Reg_E::canId)
	// 	.value("canBaudrate", mab::Md80Reg_E::canBaudrate)
	// 	.value("canWatchdog", mab::Md80Reg_E::canWatchdog)
	// 	.value("canTermination", mab::Md80Reg_E::canTermination)
	// 	.value("motorName", mab::Md80Reg_E::motorName)
	// 	.value("motorPolePairs", mab::Md80Reg_E::motorPolePairs)
	// 	.value("motorKt", mab::Md80Reg_E::motorKt)
	// 	.value("motorKt_a", mab::Md80Reg_E::motorKt_a)
	// 	.value("motorKt_b", mab::Md80Reg_E::motorKt_b)
	// 	.value("motorKt_c", mab::Md80Reg_E::motorKt_c)
	// 	.value("motorIMax", mab::Md80Reg_E::motorIMax)
	// 	.value("motorGearRatio", mab::Md80Reg_E::motorGearRatio)
	// 	.value("motorTorgueBandwidth", mab::Md80Reg_E::motorTorgueBandwidth)
	// 	.value("motorFriction", mab::Md80Reg_E::motorFriction)
	// 	.value("motorStiction", mab::Md80Reg_E::motorStiction)
	// 	.value("motorResistance", mab::Md80Reg_E::motorResistance)
	// 	.value("motorInductance", mab::Md80Reg_E::motorInductance)
	// 	.value("motorKV", mab::Md80Reg_E::motorKV)
	// 	.value("outputEncoder", mab::Md80Reg_E::outputEncoder)
	// 	.value("outputEncoderDir", mab::Md80Reg_E::outputEncoderDir)
	// 	.value("outputEncoderDefaultBaud", mab::Md80Reg_E::outputEncoderDefaultBaud)
	// 	.value("motorPosPidKp", mab::Md80Reg_E::motorPosPidKp)
	// 	.value("motorPosPidKi", mab::Md80Reg_E::motorPosPidKi)
	// 	.value("motorPosPidKd", mab::Md80Reg_E::motorPosPidKd)
	// 	.value("motorPosPidOutMax", mab::Md80Reg_E::motorPosPidOutMax)
	// 	.value("motorPosPidWindup", mab::Md80Reg_E::motorPosPidWindup)
	// 	.value("motorVelPidKp", mab::Md80Reg_E::motorVelPidKp)
	// 	.value("motorVelPidKi", mab::Md80Reg_E::motorVelPidKi)
	// 	.value("motorVelPidKd", mab::Md80Reg_E::motorVelPidKd)
	// 	.value("motorVelPidOutMax", mab::Md80Reg_E::motorVelPidOutMax)
	// 	.value("motorVelPidWindup", mab::Md80Reg_E::motorVelPidWindup)
	// 	.value("motorImpPidKp", mab::Md80Reg_E::motorImpPidKp)
	// 	.value("motorImpPidKd", mab::Md80Reg_E::motorImpPidKd)
	// 	.value("motorImpPidOutMax", mab::Md80Reg_E::motorImpPidOutMax)
	// 	.value("buildDate", mab::Md80Reg_E::buildDate)
	// 	.value("commitHash", mab::Md80Reg_E::commitHash)
	// 	.value("firmwareVersion", mab::Md80Reg_E::firmwareVersion)
	// 	.value("hardwareVersion", mab::Md80Reg_E::hardwareVersion)
	// 	.value("bridgeType", mab::Md80Reg_E::bridgeType)
	// 	.value("errorVector", mab::Md80Reg_E::errorVector)
	// 	.value("mosfetTemperature", mab::Md80Reg_E::mosfetTemperature)
	// 	.value("motorTemperature ", mab::Md80Reg_E::motorTemperature)
	// 	.export_values();

	// py::class_<mab::ImpedanceControllerGains_t>(m, "ImpedanceControllerGains_t")
	// 	.def(py::init())
	// 	.def_readwrite("kp", &mab::ImpedanceControllerGains_t::kp)
	// 	.def_readwrite("kd", &mab::ImpedanceControllerGains_t::kd)
	// 	.def_readwrite("outMax", &mab::ImpedanceControllerGains_t::outMax);

	// py::class_<mab::PidControllerGains_t>(m, "PidControllerGains_t")
	// 	.def(py::init())
	// 	.def_readwrite("kp", &mab::PidControllerGains_t::kp)
	// 	.def_readwrite("ki", &mab::PidControllerGains_t::ki)
	// 	.def_readwrite("kd", &mab::PidControllerGains_t::kd)
	// 	.def_readwrite("intWindup", &mab::PidControllerGains_t::intWindup)
	// 	.def_readwrite("outMax", &mab::PidControllerGains_t::outMax);

	// py::class_<mab::regRO_st>(m, "regRO_st")
	// 	.def(py::init())
	// 	.def_readwrite("firmwareVersion", &mab::regRO_st::firmwareVersion)
	// 	.def_readwrite("hardwareVersion", &mab::regRO_st::hardwareVersion)
	// 	.def_readwrite("buildDate", &mab::regRO_st::buildDate)
	// 	.def_readwrite("commitHash", &mab::regRO_st::commitHash)
	// 	.def_readwrite("bridgeType", &mab::regRO_st::bridgeType)
	// 	.def_readwrite("resistance", &mab::regRO_st::resistance)
	// 	.def_readwrite("inductance", &mab::regRO_st::inductance)
	// 	.def_readwrite("errorVector", &mab::regRO_st::errorVector)
	// 	.def_readwrite("mosfetTemperature", &mab::regRO_st::mosfetTemperature)
	// 	.def_readwrite("motorTemperature", &mab::regRO_st::motorTemperature);

	// py::class_<mab::regRW_st>(m, "regRW_st")
	// 	.def(py::init())
	// 	.def_readwrite("canId", &mab::regRW_st::canId)
	// 	.def_readwrite("canBaudrate", &mab::regRW_st::canBaudrate)
	// 	.def_readwrite("canWatchdog", &mab::regRW_st::canWatchdog)
	// 	.def_readwrite("canTermination", &mab::regRW_st::canTermination)
	// 	.def_readwrite("polePairs", &mab::regRW_st::polePairs)
	// 	.def_readwrite("motorKV", &mab::regRW_st::motorKV)
	// 	.def_readwrite("motorKt", &mab::regRW_st::motorKt)
	// 	.def_readwrite("motorKt_a", &mab::regRW_st::motorKt_a)
	// 	.def_readwrite("motorKt_b", &mab::regRW_st::motorKt_b)
	// 	.def_readwrite("motorKt_c", &mab::regRW_st::motorKt_c)
	// 	.def_readwrite("iMax", &mab::regRW_st::iMax)
	// 	.def_readwrite("gearRatio", &mab::regRW_st::gearRatio)
	// 	.def_readwrite("outputEncoder", &mab::regRW_st::outputEncoder)
	// 	.def_readwrite("outputEncoderDir", &mab::regRW_st::outputEncoderDir)
	// 	.def_readwrite("torqueBandwidth", &mab::regRW_st::torqueBandwidth)
	// 	.def_readwrite("friction", &mab::regRW_st::friction)
	// 	.def_readwrite("stiction", &mab::regRW_st::stiction)
	// 	.def_readwrite("impedancePdGains", &mab::regRW_st::impedancePdGains)
	// 	.def_readwrite("velocityPidGains", &mab::regRW_st::velocityPidGains)
	// 	.def_readwrite("positionPidGains", &mab::regRW_st::positionPidGains);

	// py::class_<mab::regWrite_st>(m, "regWrite_st")
	// 	.def(py::init())
	// 	.def_readwrite("RW", &mab::regWrite_st::RW);

	// py::class_<mab::regRead_st>(m, "regRead_st")
	// 	.def(py::init())
	// 	.def_readwrite("RO", &mab::regRead_st::RO)
	// 	.def_readwrite("RW", &mab::regRead_st::RW);

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
		.def("getTorque", &mab::Md80::getTorque)
		.def("getTemperature", &mab::Md80::getTemperature);
	// .def("getReadReg", &mab::Md80::getReadReg)
	// .def("getWriteReg", &mab::Md80::getWriteReg);

	py::class_<mab::Candle>(m, "Candle")
		.def(py::init<mab::CANdleBaudrate_E, bool>())
		.def(py::init<mab::CANdleBaudrate_E, bool, mab::BusType_E>())
		.def("getVersion", &mab::Candle::getVersion)
		.def("getDeviceId", &mab::Candle::getDeviceId)
		.def_readwrite("md80s", &mab::Candle::md80s)
		.def("setVebose", &mab::Candle::setVebose)
		.def("getActualCommunicationFrequency", &mab::Candle::getActualCommunicationFrequency)
		.def("setTransmitDelayUs", &mab::Candle::setTransmitDelayUs)
		.def("ping", py::overload_cast<>(&mab::Candle::ping))
		.def("ping", py::overload_cast<mab::CANdleBaudrate_E>(&mab::Candle::ping))
		.def("sengGenericFDCanFrame", &mab::Candle::sengGenericFDCanFrame)
		.def("addMd80", &mab::Candle::addMd80, py::arg("canID"), py::arg("printFailure") = false)
		.def("configCandleBaudrate", &mab::Candle::configCandleBaudrate)
		.def("configMd80Can", &mab::Candle::configMd80Can)
		.def("configMd80SetCurrentLimit", &mab::Candle::configMd80SetCurrentLimit)
		.def("configMd80Save", &mab::Candle::configMd80Save)
		.def("configMd80Blink", &mab::Candle::configMd80Blink)
		.def("configMd80TorqueBandwidth", &mab::Candle::configMd80TorqueBandwidth)
		.def("controlMd80SetEncoderZero", py::overload_cast<mab::Md80&>(&mab::Candle::controlMd80SetEncoderZero))
		.def("controlMd80SetEncoderZero", py::overload_cast<uint16_t>(&mab::Candle::controlMd80SetEncoderZero))
		.def("controlMd80Mode", py::overload_cast<mab::Md80&, mab::Md80Mode_E>(&mab::Candle::controlMd80Mode))
		.def("controlMd80Mode", py::overload_cast<uint16_t, mab::Md80Mode_E>(&mab::Candle::controlMd80Mode))
		.def("controlMd80Enable", py::overload_cast<mab::Md80&, bool>(&mab::Candle::controlMd80Enable))
		.def("controlMd80Enable", py::overload_cast<uint16_t, bool>(&mab::Candle::controlMd80Enable))
		.def("getMd80FromList", &mab::Candle::getMd80FromList)
		.def("begin", &mab::Candle::begin)
		.def("end", &mab::Candle::end)
		.def("reset", &mab::Candle::reset)
		.def("setupMd80Calibration", &mab::Candle::setupMd80Calibration)
		.def("setupMd80Diagnostic", &mab::Candle::setupMd80Diagnostic)
		.def("getCurrentBaudrate", &mab::Candle::getCurrentBaudrate)
		.def("checkMd80ForBaudrate", &mab::Candle::checkMd80ForBaudrate);
	// .def("readMd80Register", &mab::Candle::readMd80Register_<float>)
	// .def("readMd80Register", &mab::Candle::readMd80Register_<uint8_t>)
	// .def("readMd80Register", &mab::Candle::readMd80Register_<int>)
	// .def("readMd80Register", &mab::Candle::readMd80Register_<int16_t>)
	// .def("readMd80Register", &mab::Candle::readMd80Register_<uint16_t>)
	// .def("readMd80Register", &mab::Candle::readMd80Register_<int32_t>)
	// .def("readMd80Register", &mab::Candle::readMd80Register_<uint32_t>);
}
