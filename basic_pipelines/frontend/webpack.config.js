const path = require('path');

module.exports = {
  mode: 'development', // ou 'production' pour une version minifiée
  entry: './src/index.js',  // point d'entrée de votre code
  output: {
    filename: 'webrtc.bundle.js',
    path: path.resolve(__dirname, 'dist')  // le fichier sera généré dans frontend/dist/
  },
  module: {
    rules: [
      {
            test: /\.js$/,
            exclude: /node_modules/,
            use: {
            loader: 'babel-loader',
            options: {
            presets: ['@babel/preset-env']
            }
            }
      }
    ]
  },
  resolve: {
    // Pour que Webpack trouve correctement vos modules.
    extensions: ['.js']
  }
};
