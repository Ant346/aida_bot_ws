#!/bin/bash

# Скрипт для проверки команд с джойстика в Docker

echo "🎮 Проверка команд с джойстика DS4"
echo ""
echo "Вариант 1: Зайти в dev контейнер и проверить вручную"
echo "  docker compose run --rm ds4_driver_dev"
echo ""
echo "Вариант 2: Запустить DS4 driver и проверить топики"
echo "  docker compose up ds4driver"
echo ""
echo "В другом терминале:"
echo "  docker compose exec ds4driver ros2 topic echo /cmd_vel"
echo ""
echo "Или зайти в контейнер:"
echo "  docker compose exec ds4driver /bin/zsh"
echo ""

# Проверяем, запущен ли контейнер
if docker compose ps | grep -q ds4driver; then
    echo "✅ DS4 driver контейнер запущен"
    echo ""
    echo "Для проверки команд выполните в другом терминале:"
    echo "  docker compose exec ds4driver ros2 topic echo /cmd_vel"
    echo ""
    echo "Или зайдите в контейнер:"
    echo "  docker compose exec ds4driver /bin/zsh"
    echo "  ros2 topic echo /cmd_vel"
else
    echo "⚠️  DS4 driver контейнер не запущен"
    echo ""
    echo "Запустите его:"
    echo "  docker compose up -d ds4driver"
    echo ""
    echo "Или зайдите в dev контейнер:"
    echo "  docker compose run --rm ds4_driver_dev"
fi

