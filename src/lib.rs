
#![no_std]

pub mod peripherals_pixracer;
pub use peripherals_pixracer as peripherals;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
