// <<<<<<< SEARCH
pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
// =======
pub mod ffi;
pub mod solver;

// 🛡️ 向上层暴露安全的规控接口
pub use solver::预测控制求解器;
// >>>>>>> REPLACE