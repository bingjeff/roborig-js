const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const CopyWebpackPlugin = require('copy-webpack-plugin');

module.exports = {
    entry: path.join(__dirname, "src/app/index.js"), // webpack entry point. Module to start building dependency graph
    output: {
        path: path.join(__dirname, 'dist'), // Folder to store generated bundle
        filename: '[name].bundle.js',  // Name of generated bundle after build
        publicPath: '' // public URL of the output directory when referenced in a browser
    },
    module: {  // where we defined file patterns and their loaders
        rules: [
            {
                test: /\.js$/,
                use: 'babel-loader',
                exclude: [
                    /node_modules/
                ]
            }
        ]
    },
    plugins: [  // Array of plugins to apply to build chunk
        new HtmlWebpackPlugin({
            template: path.join(__dirname, "src/public/index.html"),
            inject: 'body'
        }),
        new CopyWebpackPlugin([
            { from: path.join(__dirname, "src/public/xml"), to: "xml" }
        ])
    ],
    mode: "development",
    devServer: {  // configuration for webpack-dev-server
        contentBase: './dist',  //source of static assets
        port: 8000, // port to run dev-server
    }
};