use embedded_graphics::{pixelcolor::Rgb565, prelude::Point, Drawable};

pub struct Cube {
    pub center: (f32, f32, f32),
    pub size: f32,
    pub color: Rgb565,
}

impl Drawable for Cube {
    type Color = embedded_graphics::pixelcolor::Rgb565;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: embedded_graphics::prelude::DrawTarget<Color = Self::Color>,
    {
        // Define the 8 corners and 1 point on each edge of the cube
        const POINTS: [(f32, f32, f32); 20] = [
            // Corners
            (-1.0, -1.0, -1.0),
            (1.0, -1.0, -1.0),
            (1.0, 1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
            (1.0, -1.0, 1.0),
            (1.0, 1.0, 1.0),
            (-1.0, 1.0, 1.0),
            // Edge midpoints
            (0.0, -1.0, -1.0),
            (1.0, 0.0, -1.0),
            (0.0, 1.0, -1.0),
            (-1.0, 0.0, -1.0),
            (0.0, -1.0, 1.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
            (-1.0, 0.0, 1.0),
            (-1.0, -1.0, 0.0),
            (1.0, -1.0, 0.0),
            (1.0, 1.0, 0.0),
            (-1.0, 1.0, 0.0),
        ];

        // Draw the corners of the cube
        let half_size = self.size / 2.0;
        target.draw_iter(POINTS.iter().filter_map(|(x, y, z)| {
            let x = self.center.0 + x * half_size;
            let y = self.center.1 + y * half_size;
            let z = self.center.2 + z * half_size;

            if z > 0.0 {
                let x = (100.0 * x / (z + 1.0)) as i32 + 120;
                let y = (100.0 * y / (z + 1.0)) as i32 + 160;
                if x >= 0 && x < 240 && y >= 0 && y < 320 {
                    Some(embedded_graphics::Pixel(Point::new(x, y), self.color))
                } else {
                    None
                }
            } else {
                None
            }
        }))
    }
}
