#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum CollisionShape<T> {
    None,
    /// A rectangle collider with width and height
    Rectangle { width: T, height: T },
    Circle { radius: T },
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub struct GameObject<T> {
    // Center position
    pub position: (T, T),
    // Collider shape
    pub collider: CollisionShape<T>,
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum Direction {
    Left,
    Right,
    Up,
    Down,
}

impl Direction {
    pub fn opposite(&self) -> Self {
        match self {
            Direction::Left => Direction::Right,
            Direction::Right => Direction::Left,
            Direction::Up => Direction::Down,
            Direction::Down => Direction::Up,
        }
    }

    pub fn push<T>(&self, point: (T, T)) -> (T, T)
    where
        T: core::ops::Add<Output = T> + From<i8> + Copy,
    {
        match self {
            Direction::Left => (point.0 + T::from(-1), point.1),
            Direction::Right => (point.0 + T::from(1), point.1),
            Direction::Up => (point.0, point.1 + T::from(-1)),
            Direction::Down => (point.0, point.1 + T::from(1)),
        }
    }
}

impl GameObject<i32> {
    pub fn collides_with(&self, other: &GameObject<i32>) -> bool {
        match (self.collider, other.collider) {
            (CollisionShape::Rectangle { width: w1, height: h1 }, CollisionShape::Rectangle { width: w2, height: h2 }) => {
                let left1 = self.position.0 - w1 / 2;
                let right1 = self.position.0 + w1 / 2;
                let top1 = self.position.1 - h1 / 2;
                let bottom1 = self.position.1 + h1 / 2;

                let left2 = other.position.0 - w2 / 2;
                let right2 = other.position.0 + w2 / 2;
                let top2 = other.position.1 - h2 / 2;
                let bottom2 = other.position.1 + h2 / 2;

                !(left1 >= right2 || right1 <= left2 || top1 >= bottom2 || bottom1 <= top2)
            },
            (CollisionShape::Circle { radius: r1 }, CollisionShape::Circle { radius: r2 }) => {
                let dx = self.position.0 - other.position.0;
                let dy = self.position.1 - other.position.1;
                let distance_sq = dx * dx + dy * dy;
                let radius_sum = r1 + r2;
                distance_sq < radius_sum * radius_sum
            },
            (CollisionShape::Rectangle { width: w, height: h }, CollisionShape::Circle { radius: r }) |
            (CollisionShape::Circle { radius: r }, CollisionShape::Rectangle { width: w, height: h }) => {
                let rect_center_x = self.position.0;
                let rect_center_y = self.position.1;
                let circle_center_x = other.position.0;
                let circle_center_y = other.position.1;

                let closest_x = circle_center_x.clamp(rect_center_x - w / 2, rect_center_x + w / 2);
                let closest_y = circle_center_y.clamp(rect_center_y - h / 2, rect_center_y + h / 2);

                let dx = circle_center_x - closest_x;
                let dy = circle_center_y - closest_y;

                (dx * dx + dy * dy) < (r * r)
            },
            (CollisionShape::None, _) | (_, CollisionShape::None) => false,
        }
    }
}

impl GameObject<f32> {
    pub fn collides_with(&self, other: &GameObject<f32>) -> bool {
        match (self.collider, other.collider) {
            (CollisionShape::Rectangle { width: w1, height: h1 }, CollisionShape::Rectangle { width: w2, height: h2 }) => {
                let left1 = self.position.0 - w1 / 2.0;
                let right1 = self.position.0 + w1 / 2.0;
                let top1 = self.position.1 - h1 / 2.0;
                let bottom1 = self.position.1 + h1 / 2.0;

                let left2 = other.position.0 - w2 / 2.0;
                let right2 = other.position.0 + w2 / 2.0;
                let top2 = other.position.1 - h2 / 2.0;
                let bottom2 = other.position.1 + h2 / 2.0;

                !(left1 >= right2 || right1 <= left2 || top1 >= bottom2 || bottom1 <= top2)
            },
            (CollisionShape::Circle { radius: r1 }, CollisionShape::Circle { radius: r2 }) => {
                let dx = self.position.0 - other.position.0;
                let dy = self.position.1 - other.position.1;
                let distance_sq = dx * dx + dy * dy;
                let radius_sum = r1 + r2;
                distance_sq < radius_sum * radius_sum
            },
            (CollisionShape::Rectangle { width: w, height: h }, CollisionShape::Circle { radius: r }) |
            (CollisionShape::Circle { radius: r }, CollisionShape::Rectangle { width: w, height: h }) => {
                let rect_center_x = self.position.0;
                let rect_center_y = self.position.1;
                let circle_center_x = other.position.0;
                let circle_center_y = other.position.1;

                let closest_x = circle_center_x.clamp(rect_center_x - w / 2.0, rect_center_x + w / 2.0);
                let closest_y = circle_center_y.clamp(rect_center_y - h / 2.0, rect_center_y + h / 2.0);

                let dx = circle_center_x - closest_x;
                let dy = circle_center_y - closest_y;

                (dx * dx + dy * dy) < (r * r)
            },
            (CollisionShape::None, _) | (_, CollisionShape::None) => false,
        }
    }
}