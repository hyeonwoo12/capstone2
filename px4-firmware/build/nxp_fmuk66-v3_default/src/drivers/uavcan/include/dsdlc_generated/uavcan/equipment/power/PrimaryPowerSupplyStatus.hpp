/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/hwpark/capstone2/px4-firmware/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/power/1090.PrimaryPowerSupplyStatus.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Primary power supply status.
# Typical publishing rate should be around 1~2 Hz.
#

#
# How many hours left to full discharge at average load over the last 10 seconds.
#
float16 hours_to_empty_at_10sec_avg_power               # [Hours]
float16 hours_to_empty_at_10sec_avg_power_variance      # [Hours^2]

#
# True if the publishing node senses that an external power source can be used, e.g. to charge batteries.
#
bool external_power_available

#
# Remaining energy estimate in percent.
#
uint7 remaining_energy_pct              # [Percent]     Required
uint7 remaining_energy_pct_stdev        # [Percent]     Error standard deviation. Use best guess if unknown.
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.power.PrimaryPowerSupplyStatus
saturated float16 hours_to_empty_at_10sec_avg_power
saturated float16 hours_to_empty_at_10sec_avg_power_variance
saturated bool external_power_available
saturated uint7 remaining_energy_pct
saturated uint7 remaining_energy_pct_stdev
******************************************************************************/

#undef hours_to_empty_at_10sec_avg_power
#undef hours_to_empty_at_10sec_avg_power_variance
#undef external_power_available
#undef remaining_energy_pct
#undef remaining_energy_pct_stdev

namespace uavcan
{
namespace equipment
{
namespace power
{

template <int _tmpl>
struct UAVCAN_EXPORT PrimaryPowerSupplyStatus_
{
    typedef const PrimaryPowerSupplyStatus_<_tmpl>& ParameterType;
    typedef PrimaryPowerSupplyStatus_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > hours_to_empty_at_10sec_avg_power;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > hours_to_empty_at_10sec_avg_power_variance;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > external_power_available;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > remaining_energy_pct;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > remaining_energy_pct_stdev;
    };

    enum
    {
        MinBitLen
            = FieldTypes::hours_to_empty_at_10sec_avg_power::MinBitLen
            + FieldTypes::hours_to_empty_at_10sec_avg_power_variance::MinBitLen
            + FieldTypes::external_power_available::MinBitLen
            + FieldTypes::remaining_energy_pct::MinBitLen
            + FieldTypes::remaining_energy_pct_stdev::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::hours_to_empty_at_10sec_avg_power::MaxBitLen
            + FieldTypes::hours_to_empty_at_10sec_avg_power_variance::MaxBitLen
            + FieldTypes::external_power_available::MaxBitLen
            + FieldTypes::remaining_energy_pct::MaxBitLen
            + FieldTypes::remaining_energy_pct_stdev::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::hours_to_empty_at_10sec_avg_power >::Type hours_to_empty_at_10sec_avg_power;
    typename ::uavcan::StorageType< typename FieldTypes::hours_to_empty_at_10sec_avg_power_variance >::Type hours_to_empty_at_10sec_avg_power_variance;
    typename ::uavcan::StorageType< typename FieldTypes::external_power_available >::Type external_power_available;
    typename ::uavcan::StorageType< typename FieldTypes::remaining_energy_pct >::Type remaining_energy_pct;
    typename ::uavcan::StorageType< typename FieldTypes::remaining_energy_pct_stdev >::Type remaining_energy_pct_stdev;

    PrimaryPowerSupplyStatus_()
        : hours_to_empty_at_10sec_avg_power()
        , hours_to_empty_at_10sec_avg_power_variance()
        , external_power_available()
        , remaining_energy_pct()
        , remaining_energy_pct_stdev()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<47 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1090 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.power.PrimaryPowerSupplyStatus";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool PrimaryPowerSupplyStatus_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        hours_to_empty_at_10sec_avg_power == rhs.hours_to_empty_at_10sec_avg_power &&
        hours_to_empty_at_10sec_avg_power_variance == rhs.hours_to_empty_at_10sec_avg_power_variance &&
        external_power_available == rhs.external_power_available &&
        remaining_energy_pct == rhs.remaining_energy_pct &&
        remaining_energy_pct_stdev == rhs.remaining_energy_pct_stdev;
}

template <int _tmpl>
bool PrimaryPowerSupplyStatus_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(hours_to_empty_at_10sec_avg_power, rhs.hours_to_empty_at_10sec_avg_power) &&
        ::uavcan::areClose(hours_to_empty_at_10sec_avg_power_variance, rhs.hours_to_empty_at_10sec_avg_power_variance) &&
        ::uavcan::areClose(external_power_available, rhs.external_power_available) &&
        ::uavcan::areClose(remaining_energy_pct, rhs.remaining_energy_pct) &&
        ::uavcan::areClose(remaining_energy_pct_stdev, rhs.remaining_energy_pct_stdev);
}

template <int _tmpl>
int PrimaryPowerSupplyStatus_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::hours_to_empty_at_10sec_avg_power::encode(self.hours_to_empty_at_10sec_avg_power, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::hours_to_empty_at_10sec_avg_power_variance::encode(self.hours_to_empty_at_10sec_avg_power_variance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::external_power_available::encode(self.external_power_available, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::remaining_energy_pct::encode(self.remaining_energy_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::remaining_energy_pct_stdev::encode(self.remaining_energy_pct_stdev, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int PrimaryPowerSupplyStatus_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::hours_to_empty_at_10sec_avg_power::decode(self.hours_to_empty_at_10sec_avg_power, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::hours_to_empty_at_10sec_avg_power_variance::decode(self.hours_to_empty_at_10sec_avg_power_variance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::external_power_available::decode(self.external_power_available, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::remaining_energy_pct::decode(self.remaining_energy_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::remaining_energy_pct_stdev::decode(self.remaining_energy_pct_stdev, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature PrimaryPowerSupplyStatus_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xBBA05074AD757480ULL);

    FieldTypes::hours_to_empty_at_10sec_avg_power::extendDataTypeSignature(signature);
    FieldTypes::hours_to_empty_at_10sec_avg_power_variance::extendDataTypeSignature(signature);
    FieldTypes::external_power_available::extendDataTypeSignature(signature);
    FieldTypes::remaining_energy_pct::extendDataTypeSignature(signature);
    FieldTypes::remaining_energy_pct_stdev::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef PrimaryPowerSupplyStatus_<0> PrimaryPowerSupplyStatus;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::power::PrimaryPowerSupplyStatus > _uavcan_gdtr_registrator_PrimaryPowerSupplyStatus;

}

} // Namespace power
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::power::PrimaryPowerSupplyStatus::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus >::stream(Stream& s, ::uavcan::equipment::power::PrimaryPowerSupplyStatus::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "hours_to_empty_at_10sec_avg_power: ";
    YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus::FieldTypes::hours_to_empty_at_10sec_avg_power >::stream(s, obj.hours_to_empty_at_10sec_avg_power, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "hours_to_empty_at_10sec_avg_power_variance: ";
    YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus::FieldTypes::hours_to_empty_at_10sec_avg_power_variance >::stream(s, obj.hours_to_empty_at_10sec_avg_power_variance, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "external_power_available: ";
    YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus::FieldTypes::external_power_available >::stream(s, obj.external_power_available, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "remaining_energy_pct: ";
    YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus::FieldTypes::remaining_energy_pct >::stream(s, obj.remaining_energy_pct, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "remaining_energy_pct_stdev: ";
    YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus::FieldTypes::remaining_energy_pct_stdev >::stream(s, obj.remaining_energy_pct_stdev, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace power
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::power::PrimaryPowerSupplyStatus::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::power::PrimaryPowerSupplyStatus >::stream(s, obj, 0);
    return s;
}

} // Namespace power
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_HPP_INCLUDED