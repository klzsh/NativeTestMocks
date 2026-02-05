#ifndef UNIT_TEST_SENSORS_H
#define UNIT_TEST_SENSORS_H

#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Mag/Mag.h>
#include <Sensors/IMU/IMU6DoF.h>
#include <Math/Vector.h>
#include <Math/Quaternion.h>

using namespace astra;

class FakeBarometer : public Barometer
{
public:
    FakeBarometer() : Barometer(), fakeAlt(0), fakeAltSet(false)
    {
        setName("FakeBarometer");
    }
    ~FakeBarometer() {}

    void reset()
    {
        initialized = false;
    }

    int read() override
    {
        pressure = fakeP;
        temp = fakeT;
        return 0;
    }

    // Override update(double time = -1) to prevent recalculation when altitude is set directly
    int update(double time = -1) override
    {
        if (!read())
            return -1;
        // Only calculate altitude from pressure if it wasn't set directly
        if (!fakeAltSet) {
            altitudeASL = calcAltitude(pressure);
        }
        // If altitude was set directly, altitudeASL is already correct
        return 0;
    }

    // Helper to set altitude directly
    void setAltitude(double altM)
    {
        fakeAlt = altM;
        fakeAltSet = true;
        // Calculate corresponding pressure for consistency
        fakeP = 101325.0 * pow(1.0 - altM / 44330.0, 5.255);
        fakeT = 15.0 - altM * 0.0065;
        pressure = fakeP;
        temp = fakeT;
        // Directly set the altitude in the base class
        altitudeASL = altM;
    }

    void set(double p, double t)
    {
        pressure = fakeP = p;
        temp = fakeT = t;
        fakeAltSet = false;
    }

    int init() override
    {
        initialized = true;
        return 0;
    }

    double fakeP = 101325.0;  // Default to sea level
    double fakeT = 20.0;      // Default to 20C
    double fakeAlt = 0.0;
    int fakeAltSet = false;
};

class FakeGPS : public GPS
{
public:
    FakeGPS() : GPS()
    {
        setName("FakeGPS");
    }
    ~FakeGPS() {}

    void reset()
    {
        initialized = false;
    }

    int read() override {
        return 0;
    }
    void set(double lat, double lon, double alt)
    {
        position.x() = lat;
        position.y() = lon;
        position.z() = alt;
    }
    void setHeading(double h)
    {
        heading = h;
    }
    void setDateTime(int y, int m, int d, int h, int mm, int s)
    {
        year = y;
        month = m;
        day = d;
        hr = h;
        min = mm;
        sec = s;
        snprintf(tod, 12, "%02d:%02d:%02d", hr, min, sec); // size is really 9 but 12 ignores warnings about truncation. IRL it will never truncate
    }

    int init() override
    {
        initialized = true;
        return 0;
    }

    void setHasFirstFix(int fix)
    {
        hasFix = fix;
        if (fix)
            fixQual = 4;
        else
            fixQual = 0;
    }
    void setFixQual(int qual)
    {
        fixQual = qual;
    }
};

class FakeAccel : public Accel
{
public:
    FakeAccel() : Accel("FakeAccel")
    {
    }
    ~FakeAccel() {}

    int read() override
    {
        return 0;
    }

    void set(Vector<3> accel)
    {
        acc = accel;
    }

    int init() override
    {
        acc = Vector<3>{0, 0, -9.81};
        initialized = true;
        return 0;
    }

    void reset()
    {
        initialized = false;
    }
};

class FakeGyro : public Gyro
{
public:
    FakeGyro() : Gyro("FakeGyro")
    {
    }
    ~FakeGyro() {}

    int read() override
    {
        return 0;
    }

    void set(Vector<3> gyro)
    {
        angVel = gyro;
    }

    int init() override
    {
        angVel = Vector<3>{0, 0, 0};
        initialized = true;
        return 0;
    }

    void reset()
    {
        initialized = false;
    }
};

class FakeMag : public Mag
{
public:
    FakeMag() : Mag("FakeMag")
    {
    }
    ~FakeMag() {}

    int read() override
    {
        return 0;
    }

    void set(Vector<3> magField)
    {
        mag = magField;
    }

    int init() override
    {
        mag = Vector<3>{0, 0, 0};
        initialized = true;
        return 0;
    }

    void reset()
    {
        initialized = false;
    }
};

class FakeIMU : public IMU6DoF
{
public:
    FakeIMU() : IMU6DoF("FakeIMU")
    {
    }
    ~FakeIMU() {}

    int init() override
    {
        acc = Vector<3>{0, 0, -9.81};
        angVel = Vector<3>{0, 0, 0};
        initialized = true;
        return 0;
    }

    int read() override
    {
        return 0;
    }

    void set(Vector<3> accel, Vector<3> gyro, Vector<3> mag = Vector<3>{0, 0, 0})
    {
        acc = accel;
        angVel = gyro;
        // Note: IMU6DoF doesn't have magnetometer, so mag is ignored
    }

    void reset()
    {
        initialized = false;
    }
};

#endif // UNIT_TEST_SENSORS_H
