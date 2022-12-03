use std::ffi::{c_void, CStr, CString};
use ydlidar_sdk_sys::*;

#[derive(Debug)]
struct LidarError {
    pub description: String,
}

impl LidarError {
    pub(crate) fn new(desc: &str) -> Self {
        Self {
            description: desc.to_string()
        }
    }
}

struct Ydlidar {
    lidar: *mut YDLidar,
    /*
        When it comes to string properties, the SDK will simply store the pointer passed to it.
        Which means that we need to be really careful not to give it a pointer to a string that might be dropped.
        To circumvent this, we will always point to the fields 'lidar_port' and 'ignore_array'.
    */
    lidar_port: CString,
    ignore_array: CString,
}

impl Ydlidar {
    pub fn new() -> Self {
        Self {
            lidar: unsafe { lidarCreate() },
            lidar_port: CString::default(),
            ignore_array: CString::default(),
        }
    }

    /*
        TODO: Implement some kind of builder pattern with reasonable defaults.
        Having to call set_property and handling the Result<> each time is ugly.
    */
    pub fn set_property(&mut self, prop: LidarProperty) -> Result<(), LidarError> {
        let ok = match prop {
            LidarProperty::SerialPort(str) => self.set_string_property(LidarProperty_LidarPropSerialPort, str),
            LidarProperty::IgnoreArray(str) => self.set_string_property(LidarProperty_LidarPropIgnoreArray, str),
            LidarProperty::SerialBaudRate(val) => self.set_int_property(LidarProperty_LidarPropSerialBaudrate, val),
            LidarProperty::LidarType(val) => self.set_int_property(LidarProperty_LidarPropLidarType, val),
            LidarProperty::DeviceType(val) => self.set_int_property(LidarProperty_LidarPropDeviceType, val),
            LidarProperty::SampleRate(val) => self.set_int_property(LidarProperty_LidarPropSampleRate, val),
            LidarProperty::AbnormalCheckCount(val) => self.set_int_property(LidarProperty_LidarPropAbnormalCheckCount, val),
            LidarProperty::IntensityBit(val) => self.set_int_property(LidarProperty_LidarPropIntenstiyBit, val),
            LidarProperty::MaxRange(val) => self.set_float_property(LidarProperty_LidarPropMaxRange, val),
            LidarProperty::MinRange(val) => self.set_float_property(LidarProperty_LidarPropMinRange, val),
            LidarProperty::MaxAngle(val) => self.set_float_property(LidarProperty_LidarPropMaxAngle, val),
            LidarProperty::MinAngle(val) => self.set_float_property(LidarProperty_LidarPropMinAngle, val),
            LidarProperty::ScanFrequency(val) => self.set_float_property(LidarProperty_LidarPropScanFrequency, val),
            LidarProperty::FixedResolution(val) => self.set_bool_property(LidarProperty_LidarPropFixedResolution, val),
            LidarProperty::Reversion(val) => self.set_bool_property(LidarProperty_LidarPropReversion, val),
            LidarProperty::Inverted(val) => self.set_bool_property(LidarProperty_LidarPropInverted, val),
            LidarProperty::AutoReconnect(val) => self.set_bool_property(LidarProperty_LidarPropAutoReconnect, val),
            LidarProperty::SingleChannel(val) => self.set_bool_property(LidarProperty_LidarPropSingleChannel, val),
            LidarProperty::Intensity(val) => self.set_bool_property(LidarProperty_LidarPropIntenstiy, val),
            LidarProperty::SupportMotorDtrCtrl(val) => self.set_bool_property(LidarProperty_LidarPropSupportMotorDtrCtrl, val),
            LidarProperty::SupportHeartBeat(val) => self.set_bool_property(LidarProperty_LidarPropSupportHeartBeat, val),
        };

        if !ok {
            unsafe {
                let error_description = CStr::from_ptr(DescribeError(self.lidar));
                return Err(LidarError::new(error_description.to_str().unwrap()));
            };
        }

        Ok(())
    }

    fn set_bool_property(&mut self, property_index: u32, value: bool) -> bool {
        unsafe {
            let val_ptr: *const bool = &value;
            setlidaropt(self.lidar, property_index.try_into().unwrap(), val_ptr as *const c_void, std::mem::size_of::<bool>().try_into().unwrap())
        }
    }

    fn set_float_property(&mut self, property_index: u32, value: f32) -> bool {
        unsafe {
            let val_ptr: *const f32 = &value;
            setlidaropt(self.lidar, property_index.try_into().unwrap(), val_ptr as *const c_void, std::mem::size_of::<f32>().try_into().unwrap())
        }
    }

    fn set_int_property(&mut self, property_index: u32, value: i32) -> bool {
        unsafe {
            let val_ptr: *const i32 = &value;
            setlidaropt(self.lidar, property_index.try_into().unwrap(), val_ptr as *const c_void, std::mem::size_of::<i32>().try_into().unwrap())
        }
    }

    fn set_string_property(&mut self, property_index: u32, value: &str) -> bool {
        let (string, string_len) = match property_index {
            LidarProperty_LidarPropSerialPort => {
                self.lidar_port = CString::new(value).unwrap();
                (self.lidar_port.as_ptr() as *const c_void, self.lidar_port.as_bytes().len().try_into().unwrap())
            }
            LidarProperty_LidarPropIgnoreArray => {
                self.ignore_array = CString::new(value).unwrap();
                (self.ignore_array.as_ptr() as *const c_void, self.ignore_array.as_bytes().len().try_into().unwrap())
            }
            _ => panic!("Unknown string property {}", property_index),
        };
        unsafe {
            setlidaropt(self.lidar, property_index.try_into().unwrap(), string, string_len)
        }
    }

    pub fn initialize(&mut self) -> Result<(), LidarError> {
        unsafe {
            if !initialize(self.lidar) {
                let error_description = CStr::from_ptr(DescribeError(self.lidar));
                return Err(LidarError::new(error_description.to_str().unwrap()));
            }
        }

        Ok(())
    }

    pub fn disconnect(&mut self) {
        unsafe {
            disconnecting(self.lidar);
        }
    }

    pub fn turn_on(&mut self) -> Result<(), LidarError> {
        unsafe {
            if !turnOn(self.lidar) {
                let error_description = CStr::from_ptr(DescribeError(self.lidar));
                return Err(LidarError::new(error_description.to_str().unwrap()));
            }
        }

        Ok(())
    }

    pub fn turn_off(&mut self) -> Result<(), LidarError> {
        unsafe {
            if !turnOff(self.lidar) {
                let error_description = CStr::from_ptr(DescribeError(self.lidar));
                return Err(LidarError::new(error_description.to_str().unwrap()));
            }
        }

        Ok(())
    }

    pub fn do_process_simple(&mut self) -> Result<LaserScan, LidarError> {
        let mut fan = LaserFan::default();

        unsafe {
            if !doProcessSimple(self.lidar, &mut fan as *mut _) {
                let error_description = CStr::from_ptr(DescribeError(self.lidar));
                return Err(LidarError::new(error_description.to_str().unwrap()));
            }
        };

        let npoints: usize = fan.npoints.try_into().unwrap();
        let mut points = Vec::with_capacity(npoints);

        for i in 0..fan.npoints.try_into().unwrap() {
            let laser_point = unsafe {
                let ffi_laser_point = *fan.points.add(i);
                LaserPoint::new(ffi_laser_point.angle, ffi_laser_point.range, ffi_laser_point.intensity)
            };
            points.push(laser_point);
        }

        Ok(LaserScan::new(fan.stamp, points))
    }
}


impl Drop for Ydlidar {
    fn drop(&mut self) {
        unsafe { lidarDestroy(&mut self.lidar) }
    }
}

struct LaserScan {
    stamp: u64,
    points: Vec<LaserPoint>,
}

impl LaserScan {
    pub fn new(stamp: u64, points: Vec<LaserPoint>) -> Self {
        Self {
            stamp,
            points,
        }
    }

    pub fn stamp(&self) -> u64 { self.stamp }
    pub fn points(&self) -> &Vec<LaserPoint> { &self.points }
}

struct LaserPoint {
    angle: f32,
    range: f32,
    intensity: f32,
}

impl LaserPoint {
    pub(crate) fn new(angle: f32, range: f32, intensity: f32) -> Self {
        Self {
            angle,
            range,
            intensity,
        }
    }

    pub fn angle(&self) -> f32 { self.angle }
    pub fn range(&self) -> f32 { self.range }
    pub fn intensity(&self) -> f32 { self.intensity }
}


enum LidarProperty<'a> {
    SerialPort(&'a str),
    IgnoreArray(&'a str),
    SerialBaudRate(i32),
    LidarType(i32),
    DeviceType(i32),
    SampleRate(i32),
    AbnormalCheckCount(i32),
    IntensityBit(i32),
    MaxRange(f32),
    MinRange(f32),
    MaxAngle(f32),
    MinAngle(f32),
    ScanFrequency(f32),
    FixedResolution(bool),
    Reversion(bool),
    Inverted(bool),
    AutoReconnect(bool),
    SingleChannel(bool),
    Intensity(bool),
    SupportMotorDtrCtrl(bool),
    SupportHeartBeat(bool),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_lidar() {
        let mut l = Ydlidar::new();

        l.set_property(LidarProperty::SerialPort("/dev/ydlidar")).unwrap();
        l.set_property(LidarProperty::IgnoreArray("")).unwrap();
        l.set_property(LidarProperty::SerialBaudRate(115200)).unwrap();
        l.set_property(LidarProperty::LidarType(1)).unwrap();
        l.set_property(LidarProperty::DeviceType(0)).unwrap();
        l.set_property(LidarProperty::SampleRate(3)).unwrap();
        l.set_property(LidarProperty::AbnormalCheckCount(4)).unwrap();
        l.set_property(LidarProperty::FixedResolution(true)).unwrap();
        l.set_property(LidarProperty::Reversion(false)).unwrap();
        l.set_property(LidarProperty::Inverted(true)).unwrap();
        l.set_property(LidarProperty::AutoReconnect(true)).unwrap();
        l.set_property(LidarProperty::SingleChannel(true)).unwrap();
        l.set_property(LidarProperty::Intensity(false)).unwrap();
        l.set_property(LidarProperty::SupportMotorDtrCtrl(true)).unwrap();
        l.set_property(LidarProperty::MaxAngle(180.0)).unwrap();
        l.set_property(LidarProperty::MinAngle(-180.0)).unwrap();
        l.set_property(LidarProperty::MaxRange(12.0)).unwrap();
        l.set_property(LidarProperty::MinRange(0.1)).unwrap();
        l.set_property(LidarProperty::ScanFrequency(20.0)).unwrap();

        l.initialize().unwrap();

        l.turn_on().unwrap();


        let laser_scan = l.do_process_simple().unwrap();
        assert_ne!(0, laser_scan.stamp);
        assert_ne!(0, laser_scan.points.len());

        l.turn_off().unwrap();

        l.disconnect();
    }
}