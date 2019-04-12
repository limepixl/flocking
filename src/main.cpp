#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

float magnitude(sf::Vector2f& vec)
{
    return static_cast<float>(std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2)));
}

struct Boid
{
	sf::Vector2f position;
	sf::Vector2f velocity;
	float r;	// Radius to check other boids

	Boid()
	{
		r = 25.0f;
	}

	bool operator==(const Boid& rhs)
	{
        return (position == rhs.position && velocity == rhs.velocity);
	}

	bool operator!=(const Boid& rhs)
	{
		return !(*this == rhs);
	}
};

struct Flock
{
	std::vector<Boid> boids;
    size_t numberOfBoids;

    Flock(size_t n) : numberOfBoids(n)
	{
		boids = std::vector<Boid>(n);
	}

	sf::Vector2f collision_avoidance(Boid& boid)
	{
		sf::Vector2f result;

		for(auto& b : boids)
		{
			sf::Vector2f pos = b.position;
            float dist = static_cast<float>(std::sqrt(std::pow(pos.x - boid.position.x, 2) + std::pow(pos.y - boid.position.y, 2)));

			// If the boid being looked at is within radius of myself and is not myself
			if(dist > 0 && dist <= boid.r * 2.0f)
			{
				sf::Vector2f diff = boid.position - pos;
				float mag = magnitude(diff);
				diff.x = diff.x / mag;
				diff.y = diff.y / mag;

				diff /= dist;
				result += diff;
			}
		}

		return result;
	}

	sf::Vector2f velocity_matching(Boid& boid)
	{
		sf::Vector2f result;

		for(auto& b : boids)
		{
			// Velocity matching ignores proximity
			if(boid != b)
			{
				result = result + b.velocity;
			}
		}

		result = result / static_cast<float>(numberOfBoids - 1);

		return (result - boid.velocity) / 8.0f;
	}

	sf::Vector2f flock_centering(Boid& boid)
	{
		sf::Vector2f result;

		int count = 0;
		for(auto& b : boids)
		{
			sf::Vector2f pos = b.position;
            float dist = static_cast<float>(std::sqrt(std::pow(pos.x - boid.position.x, 2) + std::pow(pos.y - boid.position.y, 2)));

			// If the boid being looked at is within radius of myself and is not myself
			if(dist > 0 && dist < boid.r)
			{
				result = result + b.position;
				count++;
			}
		}

		result = result / static_cast<float>(count + 1);

		return (result - boid.position) / 100.0f;
	}
};

int main() 
{
	const int WIDTH = 800;
	const int HEIGHT = 600;
	sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Flocking behaviour");
	window.setFramerateLimit(60);

	Flock flock(300);

	float shapeRadius = 5.0f;
	sf::CircleShape boidShape(shapeRadius);
	boidShape.setOrigin(shapeRadius, shapeRadius);

	
	for(auto& b : flock.boids)
	{
		b.position = sf::Vector2f(WIDTH / 2.0f, HEIGHT / 2.0f);
	}

	while(window.isOpen()) 
	{
		sf::Event event;
		while(window.pollEvent(event)) 
		{
			if(event.type == sf::Event::Closed)
				window.close();
		}

		window.clear();

		for(auto& b : flock.boids)
		{
			sf::Vector2f vec1 = flock.collision_avoidance(b);
			sf::Vector2f vec2 = flock.velocity_matching(b);
			sf::Vector2f vec3 = flock.flock_centering(b);

			b.velocity = b.velocity + vec1 + vec2 + vec3;

			// Limiting speed
			float speedLimit = 5.0f;
			if(magnitude(b.velocity) > speedLimit)
				b.velocity = (b.velocity / magnitude(b.velocity)) * speedLimit;

			b.position = b.position + b.velocity;

			// Wrapping around 
			auto& pos = b.position;
			if(pos.x < 0)
				pos.x = WIDTH - 1.0f;
			else if(pos.x > WIDTH)
				pos.x = 1.0f;

			if(pos.y < 0)
				pos.y = HEIGHT - 1.0f;
			else if(pos.y > HEIGHT)
				pos.y = 1.0f;

			boidShape.setPosition(b.position);
			window.draw(boidShape);
		}
		
		window.display();
	}

	return 0;
}
