
#ifndef __MEASUREMENT_H__
#define __MEASUREMENT_H__

#include <stdint.h>

enum class MeasurementType {
    UNKNOWN,
    QUATERNION,
    VECTOR_INT16
};

template<typename T>
class Measurement;

class MeasurementBase {
    public:
        template<typename T>
        void setValue(T value) {
            static_cast<Measurement<T>*>(this)->setValueImpl(value);
        }

        template<typename T>
        T getValue() const {
            return static_cast<const Measurement<T>*>(this)->getValueImpl();
        }

        virtual ~MeasurementBase() {} // Virtual destructor for polymorphic behavior
        virtual void toString(char *) const = 0;
        virtual MeasurementType getType() const = 0;

};

template<typename T>
class Measurement: public MeasurementBase {
    public:
        Measurement() {}

        void setValueImpl(T value) {
            _value = value;
        }

        T getValueImpl() const {
            return _value;
        }

        T getValue() const {
            return _value;
        }

        virtual void toString(char *) const override {};
        virtual MeasurementType getType() const override {
            return MeasurementType::UNKNOWN;
        }

    protected:
        T _value;
};

#endif