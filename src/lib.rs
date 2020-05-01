
#![no_std]

mod peripherals_pixracer;
use peripherals_pixracer as peripherals;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
