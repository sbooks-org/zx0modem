//! DOS local-time and read-only metadata used by the transfer extension.
use std::fs::{self, File, Metadata};
use std::io;
use std::os::unix::fs::PermissionsExt;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Clone, Copy, Debug)]
pub(crate) struct DosMetadata {
    time: u16,
    date: u16,
    readonly: bool,
}

fn invalid() -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, "invalid DOS file metadata")
}

fn local_time(seconds: libc::time_t) -> io::Result<libc::tm> {
    let mut result = std::mem::MaybeUninit::<libc::tm>::uninit();
    // Both pointers reference valid storage; localtime_r initializes result on success.
    if unsafe { libc::localtime_r(&seconds, result.as_mut_ptr()) }.is_null() {
        return Err(invalid());
    }
    Ok(unsafe { result.assume_init() })
}

impl DosMetadata {
    pub(crate) fn from_file(metadata: &Metadata) -> io::Result<Self> {
        let seconds = metadata
            .modified()?
            .duration_since(UNIX_EPOCH)
            .map_err(|_| invalid())?
            .as_secs();
        let local = local_time(seconds.try_into().map_err(|_| invalid())?)?;
        let year = local.tm_year + 1900;
        if !(1980..=2107).contains(&year) || !(0..=59).contains(&local.tm_sec) {
            return Err(invalid());
        }
        let result = Self {
            time: ((local.tm_hour as u16) << 11)
                | ((local.tm_min as u16) << 5)
                | (local.tm_sec as u16 / 2),
            date: (((year - 1980) as u16) << 9)
                | (((local.tm_mon + 1) as u16) << 5)
                | local.tm_mday as u16,
            readonly: metadata.permissions().mode() & 0o222 == 0,
        };
        result.validate()?;
        Ok(result)
    }

    pub(crate) fn encode(self, compression: u8) -> [u8; 8] {
        let time = self.time.to_le_bytes();
        let date = self.date.to_le_bytes();
        [
            1,
            u8::from(self.readonly),
            compression,
            0,
            time[0],
            time[1],
            date[0],
            date[1],
        ]
    }

    pub(crate) fn decode(bytes: &[u8]) -> io::Result<Self> {
        if bytes.len() != 8 || bytes[0] != 1 || bytes[3] != 0 {
            return Err(invalid());
        }
        let result = Self {
            time: u16::from_le_bytes([bytes[4], bytes[5]]),
            date: u16::from_le_bytes([bytes[6], bytes[7]]),
            readonly: bytes[1] & 1 != 0,
        };
        result.validate()?;
        Ok(result)
    }

    fn validate(self) -> io::Result<()> {
        let year = 1980 + (self.date >> 9);
        let month = (self.date >> 5) & 15;
        let day = self.date & 31;
        let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
        let days = match month {
            1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
            4 | 6 | 9 | 11 => 30,
            2 => {
                if leap {
                    29
                } else {
                    28
                }
            }
            _ => return Err(invalid()),
        };
        if day == 0
            || day > days
            || self.time >> 11 > 23
            || (self.time >> 5) & 63 > 59
            || self.time & 31 > 29
        {
            return Err(invalid());
        }
        Ok(())
    }

    pub(crate) fn modified(self) -> io::Result<SystemTime> {
        self.validate()?;
        // A zeroed tm is valid input to mktime; tm_isdst=-1 asks libc to infer DST.
        let mut local: libc::tm = unsafe { std::mem::zeroed() };
        local.tm_year = i32::from(self.date >> 9) + 80;
        local.tm_mon = i32::from((self.date >> 5) & 15) - 1;
        local.tm_mday = i32::from(self.date & 31);
        local.tm_hour = i32::from(self.time >> 11);
        local.tm_min = i32::from((self.time >> 5) & 63);
        local.tm_sec = i32::from(self.time & 31) * 2;
        local.tm_isdst = -1;
        let expected = (
            local.tm_year,
            local.tm_mon,
            local.tm_mday,
            local.tm_hour,
            local.tm_min,
            local.tm_sec,
        );
        let seconds = unsafe { libc::mktime(&mut local) };
        if seconds == -1 {
            return Err(invalid());
        }
        let actual = local_time(seconds)?;
        if expected
            != (
                actual.tm_year,
                actual.tm_mon,
                actual.tm_mday,
                actual.tm_hour,
                actual.tm_min,
                actual.tm_sec,
            )
        {
            // mktime normalizes nonexistent local times, which would silently change the date.
            return Err(invalid());
        }
        UNIX_EPOCH
            .checked_add(Duration::from_secs(
                seconds.try_into().map_err(|_| invalid())?,
            ))
            .ok_or_else(invalid)
    }

    pub(crate) fn apply_time(self, file: &File) -> io::Result<()> {
        file.set_modified(self.modified()?)
    }

    pub(crate) fn apply_readonly(self, file: &File) -> io::Result<()> {
        if self.readonly {
            let permissions = file.metadata()?.permissions();
            file.set_permissions(fs::Permissions::from_mode(permissions.mode() & !0o222))?;
        }
        // Writable destinations retain their creation permissions, including the umask.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validates_calendar_and_extension_boundaries() {
        for (year, month, day, valid) in [
            (1980, 1, 1, true),
            (2107, 12, 31, true),
            (2000, 2, 29, true),
            (2100, 2, 29, false),
            (2024, 2, 29, true),
            (2023, 2, 29, false),
            (2024, 4, 31, false),
            (2024, 0, 1, false),
            (2024, 13, 1, false),
            (2024, 1, 0, false),
        ] {
            let metadata = DosMetadata {
                date: ((year - 1980) << 9) | (month << 5) | day,
                time: (23 << 11) | (59 << 5) | 29,
                readonly: false,
            };
            assert_eq!(DosMetadata::decode(&metadata.encode(1)).is_ok(), valid);
            if valid {
                assert!(metadata.modified().is_ok());
            }
        }
        let base = DosMetadata {
            date: 33,
            time: 0,
            readonly: false,
        };
        for time in [24 << 11, 60 << 5, 30] {
            assert!(DosMetadata { time, ..base }.modified().is_err());
        }
        let bytes = base.encode(1);
        for length in 0..8 {
            assert!(DosMetadata::decode(&bytes[..length]).is_err());
        }
        for index in [0, 3] {
            let mut invalid = bytes;
            invalid[index] = 2;
            assert!(DosMetadata::decode(&invalid).is_err());
        }
        let mut extra_attributes = bytes;
        extra_attributes[1] = 0x26;
        assert!(!DosMetadata::decode(&extra_attributes).unwrap().readonly);
        extra_attributes[1] |= 1;
        assert!(DosMetadata::decode(&extra_attributes).unwrap().readonly);
    }
}
