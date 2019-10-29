#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include <random>
#include <chrono>

#define _USE_MATH_DEFINES
#include <math.h>

// Window Size
const int WindowSize = 800;

class Boid
{
private:
    sf::Vector2f acceleration;
    float size;
    float moveSpeed;
    float turnSpeed;
    float perceptionRadius;
    //float perceptionAngle;

    // Steering variables
    sf::Vector2f positionDifference;
    sf::Vector2f averageVelocity;
    sf::Vector2f averagePosition;

    sf::Vector2f separation;
    float separationWeight;
    sf::Vector2f alignment;
    float alignmentWeight;
    sf::Vector2f cohesion;
    float cohesionWeight;
    sf::Vector2f center;
    float centerWeight;
    float centerRadius;

    // Enable/Disable steering behaviors
    bool separationEnable;
    bool alignmentEnable;
    bool cohesionEnable;
    bool centerEnable;

    // Bound the Boid's position to the screen size
    sf::Vector2f edges(sf::Vector2f position)
    {
        // Bound x position
        if (position.x > WindowSize)
            position.x = 0.0f;
        else if (position.x < 0.0f)
            position.x = WindowSize;
        // Bound y position
        if (position.y > WindowSize)
            position.y = 0.0f;
        else if (position.y < 0.0f)
            position.y = WindowSize;

        return position;
    }

    // Return a normalised version of the input vector
    sf::Vector2f Normalise(sf::Vector2f vector)
    {
        float magnitude = sqrt(vector.x * vector.x + vector.y * vector.y);
        return vector / magnitude;
    }

public:
    // Shape contains color, position, etc...
    sf::CircleShape shape;
    sf::Vector2f position;
    sf::Vector2f velocity;

    // Default Constructor
    Boid()
    {
        size = 10.0f;

        position = sf::Vector2f(0.0f, 0.0f);
        velocity = sf::Vector2f(1.0f, 0.0f);
        acceleration = sf::Vector2f(0.0f, 0.0f);
        moveSpeed = 100.0f;
        turnSpeed = 0.25f;
        perceptionRadius = 50.0f;
        //perceptionAngle = 360;

        // Define a shape, currently a triangle
        shape = sf::CircleShape(size, 3);
        // Set color, move origin, and scale shape
        shape.setFillColor(sf::Color::White);
        shape.setOrigin(size / 2, size / 2);
        shape.scale(0.5f, 1.0f);
        // Set shape's position and rotation
        shape.setPosition(position);
        shape.setRotation(Direction(velocity));

        positionDifference = sf::Vector2f(0.0f, 0.0f);
        averageVelocity = sf::Vector2f(0.0f, 0.0f);
        averagePosition = sf::Vector2f(0.0f, 0.0f);

        separation = sf::Vector2f(0.0f, 0.0f);
        separationWeight = 1;
        alignment = sf::Vector2f(0.0f, 0.0f);
        alignmentWeight = 1;
        cohesion = sf::Vector2f(0.0f, 0.0f);
        cohesionWeight = 1;
        center = sf::Vector2f(0.0f, 0.0f);
        centerWeight = 1;
        centerRadius = (float)(WindowSize / 2);

        // Enable/Disable steering behaviors
        separationEnable = true;
        alignmentEnable = true;
        cohesionEnable = true;
        centerEnable = true;
    }

    // Constructor
    Boid(sf::Vector2f position, sf::Vector2f velocity)
    {
        size = 10.0f;

        this->position = position;
        this->velocity = Normalise(velocity);
        acceleration = sf::Vector2f(0.0f, 0.0f);
        moveSpeed = 175.0f;
        turnSpeed = 0.1f;
        perceptionRadius = 60.0f;
        //perceptionAngle = 360;

        // Define a shape, currently a triangle
        shape = sf::CircleShape(size, 3);
        // Set color, move origin, and scale shape
        shape.setFillColor(sf::Color::White);
        shape.setOrigin(size / 2, size / 2);
        shape.scale(0.5f, 1.0f);
        // Set shape's position and rotation
        shape.setPosition(position);
        shape.setRotation(Direction(velocity));

        separation = sf::Vector2f(0.0f, 0.0f);
        separationWeight = 1.5f;
        alignment = sf::Vector2f(0.0f, 0.0f);
        alignmentWeight = 0.25f;
        cohesion = sf::Vector2f(0.0f, 0.0f);
        cohesionWeight = 0.025f;
        center = sf::Vector2f(0.0f, 0.0f);
        centerWeight = 0.05f;
        centerRadius = (float)(WindowSize / 2);

        // Enable/Disable steering behaviors
        separationEnable = true;
        alignmentEnable = true;
        cohesionEnable = true;
        centerEnable = true;
    }

    // Update the steering vectors for all behaviors
    void Steer(std::vector<Boid> flock)
    {
        positionDifference = sf::Vector2f(0.0f, 0.0f);
        averageVelocity = sf::Vector2f(0.0f, 0.0f);
        averagePosition = sf::Vector2f(0.0f, 0.0f);
        float distance = 0;
        int total = 0;
        float distanceFromCenter = Distance(sf::Vector2f(centerRadius, centerRadius));
        float distanceWeight = pow(1.5, (distanceFromCenter / 10) - (centerRadius / 10));

        for (Boid other : flock)
        {
            distance = Distance(other.position);
            if (distance <= perceptionRadius && distance != 0)
            {
                //std::cout << "distance: " << distance << std::endl;
                // Separation math
                if (separationEnable)
                {
                    //sf::Vector2f awayFromFlockMates = (position - other.position) / distance;
                    //std::cout << "awayFromFlockMates: " << awayFromFlockMates.x << ", " << awayFromFlockMates.y << std::endl;
                    positionDifference += (position - other.position) / distance;
                }
                // Alignment math
                if(alignmentEnable) 
                {
                    averageVelocity += other.velocity;
                }
                // Cohesion math   
                if (cohesionEnable)
                {
                    averagePosition += other.position;
                }
                // Increment total Boids within perceptionRadius
                total++;
            }
        }
        if (total > 0)
        {
            positionDifference /= (float)total;
            averageVelocity /= (float)total;
            averagePosition /= (float)total;
        }
        else
        {
            positionDifference = velocity;
            averageVelocity = velocity;
            averagePosition = position;
        }

        //std::cout << "positionDifference: " << positionDifference.x << ", " << positionDifference.y << std::endl;

        // Separation
        if (separationEnable)
            separation = positionDifference - velocity;

        // Alignment
        if (alignmentEnable)
            alignment = averageVelocity - velocity;

        // Cohesion
        if (cohesionEnable)
            cohesion = averagePosition - position;

        //center
        if (centerEnable)
        {
            center = Normalise(sf::Vector2f(centerRadius, centerRadius) - position) * distanceWeight;
        }
    }

    // Update the Boids's position and rotation
    void Move(float elapsed_t, sf::RenderWindow& window)
    {
        // Update position
        acceleration += separation * separationWeight;
        acceleration += alignment * alignmentWeight;
        acceleration += cohesion * cohesionWeight;
        acceleration += center * centerWeight;
        acceleration *= turnSpeed;
        velocity = Normalise(velocity + acceleration) * elapsed_t * moveSpeed;
        position += velocity;
        // Bound position to screen
        //position = edges(position);

        shape.setPosition(position);
        shape.setRotation(Direction(velocity) + 90);
        window.draw(shape);
    }

    // Return the distance between Boid and other
    float Distance(sf::Vector2f otherPosition)
    {
        float distance = sqrt(pow(otherPosition.x - position.x, 2.0f) + pow(otherPosition.y - position.y, 2.0f));
        return distance;
    }

    // Return the direction(0, 360] of the input vector
    float Direction(sf::Vector2f vector)
    {
        float direction = atan(vector.y / vector.x) * 180.0f / (float)M_PI;

        if (vector.x < 0)
            direction += 180.0f;

        return direction;
    }

    // Return the magnitude of a vector
    float Magnitude(sf::Vector2f vector)
    {
        float magnitude = sqrt(vector.x * vector.x + vector.y * vector.y);
        return magnitude;
    }

    //Getters
    sf::Vector2f getSeparation()
    {
        return separation;
    }

    sf::Vector2f getAlignment()
    {
        return alignment;
    }

    sf::Vector2f getCohesion()
    {
        return cohesion;
    }

    float getPerceptionRadius()
    {
        return perceptionRadius;
    }

    sf::Vector2f getAverageVelocity()
    {
        return averageVelocity;
    }

    sf::Vector2f getPositionDifference()
    {
        return positionDifference;
    }
};

class Debug
{
private:
    // Enable/Disable debug options
    bool drawPerceptionRadius;
    bool drawSeparation;
    bool drawAlignment;
    bool drawCohesion;

    // Debug shapes
    sf::CircleShape radius;
    std::vector<sf::RectangleShape> localFlockMates;
    sf::RectangleShape separationLine;
    sf::RectangleShape alignLine;
    sf::CircleShape cohesionPoint;
public:
    // Constructor
    Debug(std::vector<Boid> flock, Boid& boid)
    {
        // Enable/Diable debug options
        drawPerceptionRadius = true;
        drawSeparation = true;
        drawAlignment = true;
        drawCohesion = true;

        // Highlight red
        boid.shape.setFillColor(sf::Color::Red);
        // Create Debug shapes
        if (drawPerceptionRadius)
        {
            // Circle representing perception radius
            radius = sf::CircleShape(boid.getPerceptionRadius());
            radius.setFillColor(sf::Color::Transparent);
            radius.setOutlineThickness(1.0f);
            radius.setOutlineColor(sf::Color::White);
            radius.setOrigin(boid.getPerceptionRadius(), boid.getPerceptionRadius());
        }
        if (drawSeparation)
        {
            // Line representing position difference of nearby flockmates
            separationLine = sf::RectangleShape(sf::Vector2f(boid.Magnitude(boid.getPositionDifference()), 1.0f));
            separationLine.setFillColor(sf::Color::Blue);
            separationLine.setOrigin(0.0f, 0.5f);
        }
        if (drawAlignment)
        {
            // Line representing average velocity of nearby flock mates
            alignLine = sf::RectangleShape(sf::Vector2f(boid.getPerceptionRadius(), 1.0f));
            alignLine.setFillColor(sf::Color::Green);
            alignLine.setOrigin(0.0f, 0.5f);
        }
        if (drawCohesion)
        {
            // Point representing average position of nearby flock mates
            float size = 4.0f;
            cohesionPoint = sf::CircleShape(size);
            cohesionPoint.setFillColor(sf::Color::White);
            cohesionPoint.setOrigin(size / 2, size /2);
        }
    }

    // Draw all enabled debug options
    void DrawDebug(std::vector<Boid> flock, Boid& boid, sf::RenderWindow& window)
    {
        // Perception debug
        if (drawPerceptionRadius)
        {
            radius.setPosition(boid.position);
            window.draw(radius);
        }
        // Separation debug
        if (drawSeparation)
        {
            localFlockMates.clear();
            float distance = 0;
            
            for (Boid other : flock)
            {
                distance = boid.Distance(other.position);
                if (distance <= boid.getPerceptionRadius() && distance != 0)
                {
                    sf::RectangleShape line(sf::Vector2f(distance, 1.0f));
                    line.setFillColor(sf::Color::Red);
                    line.setOrigin(0.0f, 0.5f);
                    line.setPosition(boid.position);
                    line.setRotation( boid.Direction(other.position - boid.position));
                    window.draw(line);

                    localFlockMates.push_back(line);
                }

                separationLine.setSize(sf::Vector2f(boid.Magnitude(boid.getPositionDifference()) * 10.0f, 1.0f));
                separationLine.setPosition(boid.position);
                separationLine.setRotation(boid.Direction(boid.getPositionDifference()));
                window.draw(separationLine);
            }
        }
        // Alignment debug
        if (drawAlignment)
        {
            alignLine.setPosition(boid.position);
            alignLine.setRotation(boid.Direction(boid.getAverageVelocity()));
            window.draw(alignLine);
        }
        // Cohesion debug
        if (drawCohesion)
        {
            // Draw average position of nearby flock mates
            cohesionPoint.setPosition(boid.position + boid.getCohesion());
            window.draw(cohesionPoint);
        }
        // Boid information
        std::cout << "Position: " << boid.position.x << ", " << boid.position.y << std::endl;
        std::cout << "Velocity: " << boid.velocity.x << ", " << boid.velocity.y << std::endl;
        //std::cout << "Separation: " << boid.getPositionDifference().x << ", " << boid.getPositionDifference().y << std::endl;
    }
};

int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 4;

    sf::RenderWindow window(sf::VideoMode(WindowSize, WindowSize), "Boid Simulation", sf::Style::Default, settings);
    window.setFramerateLimit(60);

    sf::Event event;

    // Clock
    sf::Clock clock;
    float elapsed_t;

    bool playSimulation = true;

    // Flock of Boids
    std::vector<Boid> flock;

    // Create a random generator for random starting positions and velocities
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> positionDistribution(0, WindowSize);
    std::uniform_real_distribution<float> velocityDistribution(-1, 1);

    // Add Boids to the flock at random positions with random velocities
    for (int i = 0; i < 60; i++)
    {
        sf::Vector2f position(positionDistribution(generator), positionDistribution(generator));
        sf::Vector2f velocity(velocityDistribution(generator), velocityDistribution(generator));
        Boid boid(position, velocity);
        flock.push_back(boid);
    }

    // Create a debug object
    Debug debug(flock, flock[0]);

    // Simulation loop
    while (window.isOpen())
    {
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;

            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::Space)
                {
                    playSimulation = !playSimulation;
                }

            case sf::Event::MouseButtonPressed:
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    sf::Vector2f position = (sf::Vector2f)sf::Mouse::getPosition(window);
                    sf::Vector2f velocity(velocityDistribution(generator), velocityDistribution(generator));
                    Boid boid(position, velocity);
                    flock.push_back(boid);
                }

            default:
                break;
            }

        }

        elapsed_t = clock.restart().asSeconds();

        if (playSimulation)
        {
            // Clear the current frame
            window.clear(sf::Color::Black);

            // Calculate all Boid movement
            for (Boid& other : flock)
            {
                other.Steer(flock);
            }

            // Move Boids
            for (Boid& other : flock)
            {
                other.Move(elapsed_t, window);
            }

            // Draw debug options
            debug.DrawDebug(flock, flock[0], window);

            // Display the current frame
            window.display();
        }
    }
    
    return EXIT_SUCCESS;
}